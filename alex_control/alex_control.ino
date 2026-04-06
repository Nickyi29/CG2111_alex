/*
 * alex_control.ino
 * CG2111A — Alex Robot
 *
 * CHANGE FROM PREVIOUS VERSION:
 *   Fixed race condition in servoUpdate() wrap detection.
 *   servoStartTick captured TCNT4 before CTC reset, causing
 *   elapsed < servoStartTick to trigger immediately and force
 *   all servo pins LOW — giving servos a 0-duration pulse.
 *   Fixed by replacing wrap detection with a simple 2.5ms timeout.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Servo / arm safety limits
// =============================================================

const int BASE_MIN     = 30,  BASE_MAX     = 150;
const int SHOULDER_MIN = 40,  SHOULDER_MAX = 140;
const int ELBOW_MIN    = 20,  ELBOW_MAX    = 160;
const int GRIPPER_MIN  = 10,  GRIPPER_MAX  = 90;

// Servo signal pins — D22-D25 = PA0-PA3
#define BASE_PIN     (1 << PA0)
#define SHOULDER_PIN (1 << PA1)
#define ELBOW_PIN    (1 << PA2)
#define GRIPPER_PIN  (1 << PA3)

volatile int          curPos[4]       = {90, 90, 90, 90};
int                   targetPos[4]    = {90, 90, 90, 90};
int                   msPerDeg        = 10;
unsigned long         lastMoveTime[4] = {0, 0, 0, 0};
volatile unsigned int servoTicks[4];

// Non-blocking servo state machine
volatile bool servoActive     = false;
volatile bool servoPinDone[4] = {false, false, false, false};

// =============================================================
// Direction constants — must match robotlib.ino
// =============================================================

#define DIR_STOP 0
#define DIR_GO   1
#define DIR_BACK 2
#define DIR_CCW  3
#define DIR_CW   4

// =============================================================
// Speed limits
// =============================================================

#define SPEED_MIN     30
#define SPEED_MAX     255
#define SPEED_DEFAULT 150
#define SPEED_STEP    50

// =============================================================
// Pin definitions
// =============================================================

// ---- E-Stop: D21 = PD0 = INT0 ----
#define ESTOP_DDR  DDRD
#define ESTOP_PORT PORTD
#define ESTOP_PINR PIND
#define ESTOP_BIT  PD0

// ---- TCS3200 control: D42-D45 = PL7-PL4 (outputs) ----
#define TCS_CTRL_DDR  DDRL
#define TCS_CTRL_PORT PORTL
#define TCS_S0_BIT    PL7
#define TCS_S1_BIT    PL6
#define TCS_S2_BIT    PL5
#define TCS_S3_BIT    PL4

// ---- TCS3200 OUT: D46 = PL3 (input, polled) ----
#define TCS_OUT_DDR  DDRL
#define TCS_OUT_PORT PORTL
#define TCS_OUT_PINR PINL
#define TCS_OUT_BIT  PL3

// =============================================================
// State
// =============================================================

volatile TState        buttonState     = STATE_RUNNING;
volatile bool          stateChanged    = false;
volatile unsigned long lastButtonIsrMs = 0;
volatile uint8_t       motorSpeed      = SPEED_DEFAULT;
volatile uint8_t       currentDir      = DIR_STOP;

// =============================================================
// E-Stop ISR — INT0 on PD0 (D21)
// =============================================================

ISR(INT0_vect) {
    unsigned long now = millis();
    if ((unsigned long)(now - lastButtonIsrMs) < 50) return;
    lastButtonIsrMs = now;

    uint8_t pressed = (ESTOP_PINR & (1 << ESTOP_BIT)) ? 0 : 1;

    if (buttonState == STATE_RUNNING && pressed) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
    } else if (buttonState == STATE_STOPPED && !pressed) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
}

// =============================================================
// Servo PWM — Timer 4 CTC, non-blocking state machine
// =============================================================

void updateTicks(void) {
    unsigned int newTicks[4];
    for (int i = 0; i < 4; i++) {
        newTicks[i] = 2000 + (unsigned int)((long)curPos[i] * 2000 / 180);
    }
    cli();
    for (int i = 0; i < 4; i++) servoTicks[i] = newTicks[i];
    sei();
}

// ISR fires every 20ms — takes ~2µs, does not block serial RX
ISR(TIMER4_COMPA_vect) {
    // All servo pins HIGH — start of pulse window
    PORTA |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

    // Reset state machine flags
    servoPinDone[0] = false;
    servoPinDone[1] = false;
    servoPinDone[2] = false;
    servoPinDone[3] = false;
    servoActive     = true;

    // ISR exits immediately — loop() handles pulling pins LOW
    // Note: servoStartTick removed — was causing race condition
    // because TCNT4 resets to 0 in CTC mode immediately after match
}

// Called every loop() iteration to complete the servo pulse.
// Uses TCNT4 directly — in CTC mode TCNT4 counts 0 to 40000 (20ms).
// servoTicks values are 2000-4000 (1ms-2ms), well within the 20ms window.
// Simple timeout at 5000 ticks (2.5ms) cleans up if loop() is slow.
void servoUpdate(void) {
    if (!servoActive) return;

    unsigned int elapsed = TCNT4;

    // Safety timeout — 5000 ticks = 2.5ms
    // Longest valid servo pulse is 2ms (4000 ticks)
    // If we are past 2.5ms, force all pins LOW and finish
    if (elapsed > 5000) {
        PORTA &= ~(BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
        servoActive = false;
        return;
    }

    // Pull each pin LOW when its tick count is reached
    if (!servoPinDone[0] && elapsed >= servoTicks[0]) {
        PORTA &= ~BASE_PIN;
        servoPinDone[0] = true;
    }
    if (!servoPi
