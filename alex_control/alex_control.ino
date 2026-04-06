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
volatile unsigned int servoTicks[4] = {3000, 3000, 3000, 3000}; // 90° default = 1500us pulse

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
    if (!servoPinDone[1] && elapsed >= servoTicks[1]) {
        PORTA &= ~SHOULDER_PIN;
        servoPinDone[1] = true;
    }
    if (!servoPinDone[2] && elapsed >= servoTicks[2]) {
        PORTA &= ~ELBOW_PIN;
        servoPinDone[2] = true;
    }
    if (!servoPinDone[3] && elapsed >= servoTicks[3]) {
        PORTA &= ~GRIPPER_PIN;
        servoPinDone[3] = true;
    }

    // All pins done — mark pulse complete
    if (servoPinDone[0] && servoPinDone[1] &&
        servoPinDone[2] && servoPinDone[3]) {
        servoActive = false;
    }
}

// =============================================================
// Arm movement — non-blocking
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
    delayMicroseconds(300);

    uint32_t      risingEdges = 0;
    uint8_t       prev        = readTcsOut();
    unsigned long startMs     = millis();

    while ((unsigned long)(millis() - startMs) < 100) {
        uint8_t cur = readTcsOut();
        if (!prev && cur) risingEdges++;
        prev = cur;
    }
    return risingEdges * 10UL;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannelHz(false, false);  // S2=L S3=L → red
    *g = measureChannelHz(true,  true);   // S2=H S3=H → green
    *b = measureChannelHz(false, true);   // S2=L S3=H → blue
}

// =============================================================
// Arm command acknowledgement helper
// Sends RESP_OK with the accepted target degree so both terminals
// can confirm the Arduino actually received and accepted the command.
// Without this, a silent E-Stop rejection looks identical to acceptance
// from the operator's perspective.
// =============================================================

static void sendArmAck(const char *joint, uint32_t targetDeg) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = RESP_OK;
    pkt.params[0]  = targetDeg;
    strncpy(pkt.data, joint, sizeof(pkt.data) - 1);
    pkt.data[sizeof(pkt.data) - 1] = '\0';
    sendFrame(&pkt);
}

// =============================================================
// Command handler
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {

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

        case COMMAND_FORWARD:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_GO;
            forward(motorSpeed);
            break;

        case COMMAND_BACKWARD:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_BACK;
            backward(motorSpeed);
            break;

        case COMMAND_LEFT:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_CCW;
            ccw(motorSpeed);
            break;

        case COMMAND_RIGHT:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_CW;
            cw(motorSpeed);
            break;

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

        case COMMAND_STOP:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_STOP;
            stop();
            break;

        case COMMAND_ARM_BASE:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[0] = constrain((int)cmd->params[0], BASE_MIN, BASE_MAX);
            sendArmAck("BASE", (uint32_t)targetPos[0]);
            break;

        case COMMAND_ARM_SHOULDER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[1] = constrain((int)cmd->params[0], SHOULDER_MIN, SHOULDER_MAX);
            sendArmAck("SHOULDER", (uint32_t)targetPos[1]);
            break;

        case COMMAND_ARM_ELBOW:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[2] = constrain((int)cmd->params[0], ELBOW_MIN, ELBOW_MAX);
            sendArmAck("ELBOW", (uint32_t)targetPos[2]);
            break;

        case COMMAND_ARM_GRIPPER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[3] = constrain((int)cmd->params[0], GRIPPER_MIN, GRIPPER_MAX);
            sendArmAck("GRIPPER", (uint32_t)targetPos[3]);
            break;

        case COMMAND_ARM_HOME:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            for (int i = 0; i < 4; i++) targetPos[i] = 90;
            sendArmAck("HOME", 90);
            break;

        case COMMAND_ARM_SPEED:
            msPerDeg = (int)cmd->params[0];
            break;

        default:
            break;
    }
}

// =============================================================
// setup()
// =============================================================

void setup(void) {
#if USE_BAREMETAL_SERIAL
    usartInit(8);
#else
    Serial.begin(115200);
#endif

    // Port A — servo signal outputs (PA0-PA3 = D22-D25)
    DDRA |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

    // Port L — TCS3200 (D42-D46 = PL7-PL3)
    DDRL |=  (1 << TCS_S0_BIT) | (1 << TCS_S1_BIT)
           | (1 << TCS_S2_BIT) | (1 << TCS_S3_BIT);
    DDRL &= ~(1 << TCS_OUT_BIT);
    PORTL &= ~(1 << TCS_OUT_BIT);
    PORTL |=  (1 << TCS_S0_BIT);
    PORTL &= ~(1 << TCS_S1_BIT);

    // E-Stop — D21 = PD0 = INT0, input with pull-up, any-change ISR
    ESTOP_DDR  &= ~(1 << ESTOP_BIT);
    ESTOP_PORT |=  (1 << ESTOP_BIT);
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |=  (1 << ISC00);
    EIMSK |=  (1 << INT0);

    // Timer 4: 16-bit CTC, 20ms period for servo PWM
    // updateTicks() is called BEFORE cli() so its internal cli/sei runs cleanly,
    // and servoTicks are valid before the very first ISR fires.
    updateTicks();
    cli();
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;
    OCR4A  = 40000;
    TCCR4B |= (1 << WGM42);
    TCCR4B |= (1 << CS41);
    TIMSK4 |= (1 << OCIE4A);
    sei();

    motorsInit();
}

// =============================================================
// loop()
// =============================================================

void loop(void) {
    // On the very first loop iteration, broadcast STATUS_RUNNING so both
    // operator terminals know the Arduino started up in a RUNNING state.
    static bool startupBroadcastDone = false;
    if (!startupBroadcastDone) {
        sendStatus(STATE_RUNNING);
        startupBroadcastDone = true;
    }

    // Complete servo pulse — non-blocking, replaces old busy-wait ISR
    servoUpdate();

    // Step arm joints towards target positions
    processMovement();

    // Handle E-Stop state changes
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        if (state == STATE_STOPPED) {
            currentDir = DIR_STOP;
            stop();
        }
        sendStatus(state);
    }

    // Receive and process one TPacket per loop iteration
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
