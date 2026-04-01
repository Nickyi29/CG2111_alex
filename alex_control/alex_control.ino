/*
 * alex_control.ino
 * CG2111A — Alex Robot
 *
 * CHANGES FROM PREVIOUS VERSION:
 *
 *  All pin changes are due to the MH Electronics Motor Shield occupying
 *  D0-D13 and A0-A5 headers on the Mega, leaving only D14-D52 and A8-A15
 *  available for external wiring.
 *
 *  1. E-Stop moved: D2 (PE4/INT4) → D21 (PD0/INT0)
 *     - D2 is under the shield header, inaccessible
 *     - D21 is free, has a proper hardware external interrupt (INT0)
 *     - ISR vector changed: INT4_vect → INT0_vect
 *     - Control registers changed: EICRB/ISC4x → EICRA/ISC0x
 *
 *  2. Servo pins moved: A0-A3 (PF0-PF3) → D22-D25 (PA0-PA3)
 *     - A0-A5 header is occupied by the shield
 *     - D22-D25 = PA0-PA3, no timer/special-function conflicts
 *     - All DDRF/PORTF servo references → DDRA/PORTA
 *     - Timer 4 ISR now bit-bangs PORTA instead of PORTF
 *
 *  3. TCS3200 control pins moved: A4-A7 (PF4-PF7) → D26-D29 (PA4-PA7)
 *     - Shares Port A with servos (PA0-PA3) — one DDRA write sets all 8
 *     - All DDRF/PORTF TCS ctrl references → DDRA/PORTA
 *
 *  4. TCS3200 OUT moved: D22 (PA0) → D30 (PC7)
 *     - D22 is now the base servo pin
 *     - D30 = PC7, free GPIO input, no conflicts
 *     - TCS_OUT references changed to DDRC/PORTC/PINC / PC7
 *
 * Final pin summary:
 *   D21 PD0 INT0   E-Stop button (input, pull-up, any-change ISR)
 *   D22 PA0        Base servo signal (output, Timer 4 bit-bang)
 *   D23 PA1        Shoulder servo signal
 *   D24 PA2        Elbow servo signal
 *   D25 PA3        Gripper servo signal
 *   D26 PA4        TCS3200 S0 (output)
 *   D27 PA5        TCS3200 S1 (output)
 *   D28 PA6        TCS3200 S2 (output)
 *   D29 PA7        TCS3200 S3 (output)
 *   D30 PC7        TCS3200 OUT (input, polled)
 *   D18 PD3 INT3   Left encoder (optional)
 *   D19 PD2 INT2   Right encoder (optional)
 *   D3  PE5 OC3C   Motor M3/M4 PWM — shield internal, Timer 3
 *   D11 PB5 OC1A   Motor M1/M2 PWM — shield internal, Timer 1
 *   D4  PG5        Shift reg CLK   — shield internal
 *   D7  PH4        Shift reg OE    — shield internal
 *   D8  PH5        Shift reg DATA  — shield internal
 *   D12 PB6        Shift reg LATCH — shield internal
 *   A8-A15         Free
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
// Bit masks within Port A
#define BASE_PIN     (1 << PA0)
#define SHOULDER_PIN (1 << PA1)
#define ELBOW_PIN    (1 << PA2)
#define GRIPPER_PIN  (1 << PA3)

volatile int          curPos[4]       = {90, 90, 90, 90};
int                   targetPos[4]    = {90, 90, 90, 90};
int                   msPerDeg        = 10;
unsigned long         lastMoveTime[4] = {0, 0, 0, 0};
volatile unsigned int servoTicks[4];

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

// ---- TCS3200 control: D26-D29 = PA4-PA7 ----
// Share Port A with servo pins (PA0-PA3) above
#define TCS_CTRL_DDR  DDRA
#define TCS_CTRL_PORT PORTA
#define TCS_S0_BIT    PA4
#define TCS_S1_BIT    PA5
#define TCS_S2_BIT    PA6
#define TCS_S3_BIT    PA7

// ---- TCS3200 OUT: D30 = PC7 (input, polled) ----
#define TCS_OUT_DDR  DDRC
#define TCS_OUT_PORT PORTC
#define TCS_OUT_PINR PINC
#define TCS_OUT_BIT  PC7

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
    if ((unsigned long)(now - lastButtonIsrMs) < 50) return;   // debounce 50 ms
    lastButtonIsrMs = now;

    // Button wired with pull-up: pressed = LOW
    uint8_t pressed = (ESTOP_PINR & (1 << ESTOP_BIT)) ? 0 : 1;

    if (buttonState == STATE_RUNNING && pressed) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
        stop();                     // cut motors immediately in ISR
    } else if (buttonState == STATE_STOPPED && !pressed) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
}

// =============================================================
// Servo PWM — Timer 4 CTC, bit-bang on Port A (D22-D25)
// =============================================================

void updateTicks(void) {
    for (int i = 0; i < 4; i++) {
        // Map 0-180 deg → 2000-4000 timer ticks (1000 µs – 2000 µs pulse)
        // Timer 4 runs at 2 MHz (prescaler 8): 1 tick = 0.5 µs
        servoTicks[i] = 2000 + (unsigned int)((long)curPos[i] * 2000 / 180);
    }
}

// Fires every 20 ms (OCR4A = 40000 @ 2 MHz)
ISR(TIMER4_COMPA_vect) {
    // All four servo pins HIGH — start of pulse window
    PORTA |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

    bool bA = true, sA = true, eA = true, gA = true;
    while (bA || sA || eA || gA) {
        unsigned int elapsed = TCNT4;
        if (elapsed >= servoTicks[0]) { PORTA &= ~BASE_PIN;     bA = false; }
        if (elapsed >= servoTicks[1]) { PORTA &= ~SHOULDER_PIN; sA = false; }
        if (elapsed >= servoTicks[2]) { PORTA &= ~ELBOW_PIN;    eA = false; }
        if (elapsed >= servoTicks[3]) { PORTA &= ~GRIPPER_PIN;  gA = false; }
    }
}

// =============================================================
// Arm movement — non-blocking, called every loop()
// =============================================================

void processMovement(void) {
    unsigned long now = millis();
    for (int i = 0; i < 4; i++) {
        if (curPos[i] != targetPos[i] &&
            (unsigned long)(now - lastMoveTime[i]) >= (unsigned long)msPerDeg) {
            if (targetPos[i] > curPos[i]) curPos[i]++;
            else                          curPos[i]--;
            lastMoveTime[i] = now;
            updateTicks();
        }
    }
}

// =============================================================
// Packet helpers
// =============================================================

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// TCS3200 helpers
// =============================================================

static inline void setTcsFilter(bool s2High, bool s3High) {
    if (s2High) TCS_CTRL_PORT |=  (1 << TCS_S2_BIT);
    else        TCS_CTRL_PORT &= ~(1 << TCS_S2_BIT);
    if (s3High) TCS_CTRL_PORT |=  (1 << TCS_S3_BIT);
    else        TCS_CTRL_PORT &= ~(1 << TCS_S3_BIT);
}

static inline uint8_t readTcsOut(void) {
    return (TCS_OUT_PINR & (1 << TCS_OUT_BIT)) ? 1 : 0;
}

static uint32_t measureChannelHz(bool s2High, bool s3High) {
    setTcsFilter(s2High, s3High);
    delayMicroseconds(300);         // settle time after filter switch

    uint32_t      risingEdges = 0;
    uint8_t       prev        = readTcsOut();
    unsigned long startMs     = millis();

    while ((unsigned long)(millis() - startMs) < 100) {
        uint8_t cur = readTcsOut();
        if (!prev && cur) risingEdges++;
        prev = cur;
    }
    return risingEdges * 10UL;      // Hz = edges × 10 for 100 ms window
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannelHz(false, false);  // S2=L S3=L → red
    *g = measureChannelHz(true,  true);   // S2=H S3=H → green
    *b = measureChannelHz(false, true);   // S2=L S3=H → blue
}

// =============================================================
// Command handler
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {

        // -------------------------------------------------------
        case COMMAND_ESTOP: {
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            currentDir = DIR_STOP;
            stop();
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_OK;
            strncpy(pkt.data, "E-Stop activated", sizeof(pkt.data) - 1);
            pkt.data[sizeof(pkt.data) - 1] = '\0';
            sendFrame(&pkt);
            sendStatus(STATE_STOPPED);
            break;
        }

        // -------------------------------------------------------
        case COMMAND_COLOR: {
            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            pkt.params[0]  = r;
            pkt.params[1]  = g;
            pkt.params[2]  = b;
            strncpy(pkt.data, "Color sample ready", sizeof(pkt.data) - 1);
            pkt.data[sizeof(pkt.data) - 1] = '\0';
            sendFrame(&pkt);
            break;
        }

        // -------------------------------------------------------
        case COMMAND_FORWARD:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_GO;
            forward(motorSpeed);
            break;

        // -------------------------------------------------------
        case COMMAND_BACKWARD:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_BACK;
            backward(motorSpeed);
            break;

        // -------------------------------------------------------
        case COMMAND_LEFT:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_CCW;
            ccw(motorSpeed);
            break;

        // -------------------------------------------------------
        case COMMAND_RIGHT:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_CW;
            cw(motorSpeed);
            break;

        // -------------------------------------------------------
        case COMMAND_SPEED: {
            int delta    = (cmd->params[0] == 1) ? SPEED_STEP : -SPEED_STEP;
            int newSpeed = (int)motorSpeed + delta;
            if (newSpeed < SPEED_MIN) newSpeed = SPEED_MIN;
            if (newSpeed > SPEED_MAX) newSpeed = SPEED_MAX;
            motorSpeed = (uint8_t)newSpeed;
            if (buttonState != STATE_STOPPED && currentDir != DIR_STOP)
                move(motorSpeed, currentDir);
            sendResponse(RESP_OK, motorSpeed);
            break;
        }

        // -------------------------------------------------------
        case COMMAND_ARM_BASE:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[0] = constrain((int)cmd->params[0], BASE_MIN, BASE_MAX);
            break;

        // -------------------------------------------------------
        case COMMAND_ARM_SHOULDER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[1] = constrain((int)cmd->params[0], SHOULDER_MIN, SHOULDER_MAX);
            break;

        // -------------------------------------------------------
        case COMMAND_ARM_ELBOW:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[2] = constrain((int)cmd->params[0], ELBOW_MIN, ELBOW_MAX);
            break;

        // -------------------------------------------------------
        case COMMAND_ARM_GRIPPER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[3] = constrain((int)cmd->params[0], GRIPPER_MIN, GRIPPER_MAX);
            break;

        // -------------------------------------------------------
        case COMMAND_ARM_HOME:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            for (int i = 0; i < 4; i++) targetPos[i] = 90;
            break;

        // -------------------------------------------------------
        case COMMAND_ARM_SPEED:
            msPerDeg = (int)cmd->params[0];
            break;

        // -------------------------------------------------------
        default:
            break;
    }
}

// =============================================================
// setup()
// =============================================================

void setup(void) {
#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    // ------------------------------------------------------------------
    // Port A — servos (PA0-PA3) + TCS3200 control (PA4-PA7), all outputs
    // One DDRA write configures all 8 bits at once
    // ------------------------------------------------------------------
    DDRA |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN)  // PA0-PA3
          | (1 << TCS_S0_BIT) | (1 << TCS_S1_BIT)               // PA4-PA5
          | (1 << TCS_S2_BIT) | (1 << TCS_S3_BIT);              // PA6-PA7

    // 20% output frequency scaling: S0=HIGH, S1=LOW
    PORTA |=  (1 << TCS_S0_BIT);
    PORTA &= ~(1 << TCS_S1_BIT);

    // ------------------------------------------------------------------
    // TCS3200 OUT — D30 = PC7 (input, no pull-up)
    // ------------------------------------------------------------------
    TCS_OUT_DDR  &= ~(1 << TCS_OUT_BIT);
    TCS_OUT_PORT &= ~(1 << TCS_OUT_BIT);

    // ------------------------------------------------------------------
    // E-Stop — D21 = PD0 = INT0
    // Input with pull-up; ISR fires on any logical change
    // ------------------------------------------------------------------
    ESTOP_DDR  &= ~(1 << ESTOP_BIT);
    ESTOP_PORT |=  (1 << ESTOP_BIT);   // enable internal pull-up

    // INT0: any logical change → ISC01=0, ISC00=1
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |=  (1 << ISC00);
    EIMSK |=  (1 << INT0);

    // ------------------------------------------------------------------
    // Timer 4: 16-bit CTC, 20 ms period for servo PWM
    // 16 MHz / prescaler 8 = 2 MHz → 1 tick = 0.5 µs
    // OCR4A = 40 000 → period = 40 000 × 0.5 µs = 20 ms ✓
    // ------------------------------------------------------------------
    cli();
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;
    OCR4A  = 40000;
    TCCR4B |= (1 << WGM42);   // CTC mode (top = OCR4A)
    TCCR4B |= (1 << CS41);    // prescaler 8
    TIMSK4 |= (1 << OCIE4A);  // enable compare-match A interrupt
    sei();

    updateTicks();

    // ------------------------------------------------------------------
    // Bare-metal motor driver
    // Sets up Timer 1 (OC1A/D11) + Timer 3 (OC3C/D3) + 74HC595 shift reg
    // All driven through shield PCB traces — no external wiring needed
    // ------------------------------------------------------------------
    motorsInit();
}

// =============================================================
// loop()
// =============================================================

void loop(void) {
    // Non-blocking servo stepping — must run every iteration
    processMovement();

    // Forward E-Stop state changes to Pi
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // Receive and handle one TPacket per loop iteration if available
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
