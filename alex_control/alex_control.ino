/*
 * alex_control.ino
 * CG2111A — Alex Robot
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
const int SHOULDER_MIN = 60,  SHOULDER_MAX = 170;  
const int ELBOW_MIN    = 10,  ELBOW_MAX    = 170;
const int GRIPPER_MIN  = 40,  GRIPPER_MAX  = 120;
// Servo signal pins — D22-D25 = PA0-PA3
#define BASE_PIN     (1 << PA0)
#define SHOULDER_PIN (1 << PA1)
#define ELBOW_PIN    (1 << PA2)
#define GRIPPER_PIN  (1 << PA3)

// Gripper boots CLOSED (10°) to prevent snap-open on power-on
volatile int          curPos[4]       = {90, 90, 90, 55};
int                   targetPos[4]    = {90, 90, 90, 55};
int                   msPerDeg        = 1;
unsigned long         lastMoveTime[4] = {0, 0, 0, 0};

// 10° = 2000 + (10 * 2000 / 180) = 2111 ticks
volatile unsigned int servoTicks[4]   = {3000, 3000, 3000, 2111};

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

#define ESTOP_DDR  DDRD
#define ESTOP_PORT PORTD
#define ESTOP_PINR PIND
#define ESTOP_BIT  PD0

#define TCS_CTRL_DDR  DDRL
#define TCS_CTRL_PORT PORTL
#define TCS_S0_BIT    PL7
#define TCS_S1_BIT    PL6
#define TCS_S2_BIT    PL5
#define TCS_S3_BIT    PL4

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
    if ((unsigned long)(now - lastButtonIsrMs) < 200) return;
    lastButtonIsrMs = now;

    uint8_t pressed = (ESTOP_PINR & (1 << ESTOP_BIT)) ? 0 : 1;

    if (pressed) {
        if (buttonState == STATE_RUNNING) {
            buttonState  = STATE_STOPPED;
            stateChanged = true;
        }
    } else {
        static uint8_t pressCount = 0;
        pressCount++;
        if (pressCount % 2 == 0) {
            buttonState  = STATE_RUNNING;
            stateChanged = true;
        }
    }
}

// =============================================================
// Servo PWM — Timer4 CTC, blocking ISR
// Busy-wait inside ISR pulls pins LOW at exact tick counts.
// Max blocking time = 2ms (longest servo pulse).
// Serial bytes are hardware-buffered so nothing is lost.
// =============================================================

void updateTicks(void) {
    cli();
    for (int i = 0; i < 4; i++) {
        servoTicks[i] = 2000 + (unsigned int)((long)curPos[i] * 2000 / 180);
    }
    sei();
}

ISR(TIMER4_COMPA_vect) {
    // --- Snapshot tick thresholds BEFORE sei() ---
    // Using local copies means updateTicks() running after sei() cannot corrupt the thresholds mid-pulse.
    unsigned int t0 = servoTicks[0];
    unsigned int t1 = servoTicks[1];
    unsigned int t2 = servoTicks[2];
    unsigned int t3 = servoTicks[3];
    // All servo pins HIGH — start of pulse window
    PORTA |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
    sei();
    // Busy-wait: pull each pin LOW at its exact tick count
    bool bActive = true, sActive = true, eActive = true, gActive = true;
    while (bActive || sActive || eActive || gActive) {
        unsigned int elapsed = TCNT4;
        if (bActive && elapsed >= t0) {
            PORTA &= ~BASE_PIN;
            bActive = false;
        }
        if (sActive && elapsed >= t1) {
            PORTA &= ~SHOULDER_PIN;
            sActive = false;
        }
        if (eActive && elapsed >= t2) {
            PORTA &= ~ELBOW_PIN;
            eActive = false;
        }
        if (gActive && elapsed >= t3) {
            PORTA &= ~GRIPPER_PIN;
            gActive = false;
        }
    }
}

// =============================================================
// Arm movement — non-blocking, steps 1 deg per msPerDeg ms
// =============================================================

void processMovement(void) {
    unsigned long now     = millis();
    bool          changed = false;

    for (int i = 0; i < 4; i++) {
        if (curPos[i] != targetPos[i] &&
            (unsigned long)(now - lastMoveTime[i]) >= (unsigned long)msPerDeg) {
            if (targetPos[i] > curPos[i]) curPos[i]++;
            else                          curPos[i]--;
            lastMoveTime[i] = now;
            changed = true;
        }
    }
    if (changed) updateTicks();
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
    *r = measureChannelHz(false, false);
    *g = measureChannelHz(true,  true);
    *b = measureChannelHz(false, true);
}

// =============================================================
// Command handler
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {

        case COMMAND_ESTOP: {
            cli();
            TState newState = (buttonState == STATE_RUNNING)
                              ? STATE_STOPPED
                              : STATE_RUNNING;
            buttonState  = newState;
            stateChanged = false;
            sei();
            if (newState == STATE_STOPPED) {
                currentDir = DIR_STOP;
                stop();
            }
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_OK;
            strncpy(pkt.data,
                    (newState == STATE_STOPPED) ? "E-Stop ON" : "E-Stop OFF",
                    sizeof(pkt.data) - 1);
            pkt.data[sizeof(pkt.data) - 1] = '\0';
            sendFrame(&pkt);
            sendStatus(newState);
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
            targetPos[0] = 90;
            targetPos[1] = 90;
            targetPos[2] = 90;
            targetPos[3] = 55;   
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

    // Port A — servo pins as outputs, immediately LOW
    // Prevents noise reaching servos during boot sequence
    DDRA  |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
    PORTA &= ~(BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

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

    // Settle delay — let servos stabilise before first PWM pulse
    updateTicks();
    delay(500);

    // Timer4: 16-bit CTC, 20ms period for servo PWM
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
    static bool startupBroadcastDone = false;
    if (!startupBroadcastDone) {
        sendStatus(STATE_RUNNING);
        startupBroadcastDone = true;
    }

    processMovement();

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

    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
