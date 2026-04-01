/*
 * alex_control.ino
 * CG2111A — Alex Robot
 *
 * CHANGES FROM ORIGINAL:
 *
 *  1. Removed #include <AFMotor.h> — fully bare-metal, motorsInit() called instead.
 *
 *  2. Servo timer moved from Timer 1 → Timer 4:
 *       TCCR1x / OCR1A / TIMSK1 / TIMER1_COMPA_vect
 *     replaced with:
 *       TCCR4x / OCR4A / TIMSK4 / TIMER4_COMPA_vect
 *     Reason: Motor shield physically uses OC1A (D11) for M1/M2 PWM.
 *
 *  3. Servo signal pins moved from PORTC (PC0-PC3 = D37-D34 on Mega)
 *     to PORTF (PF0-PF3 = A0-A3).
 *     PORTC on the Mega is NOT A0-A3.
 *
 *  4. Color sensor OUT pin moved from PE5 (D3) to PA0 (D22).
 *     PE5 = D3 = OC3C is the motor shield M3/M4 PWM pin.
 *
 *  5. Added motorsInit() call in setup().
 *
 * Pin summary:
 *   A0  PF0   Servo base signal
 *   A1  PF1   Servo shoulder signal
 *   A2  PF2   Servo elbow signal
 *   A3  PF3   Servo gripper signal
 *   A4  PF4   TCS3200 S0
 *   A5  PF5   TCS3200 S1
 *   A6  PF6   TCS3200 S2
 *   A7  PF7   TCS3200 S3
 *   D22 PA0   TCS3200 OUT (input)
 *   D2  PE4   E-Stop (INT4)
 *   D3  PE5   OC3C Motor M3/M4 PWM — DO NOT USE AS GPIO
 *   D4  PG5   Motor shield shift reg CLK
 *   D7  PH4   Motor shield shift reg OE
 *   D8  PH5   Motor shield shift reg DATA
 *   D11 PB5   OC1A Motor M1/M2 PWM — DO NOT USE AS GPIO
 *   D12 PB6   Motor shield shift reg LATCH
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Servo / arm constants
// =============================================================

const int BASE_MIN     = 30,  BASE_MAX     = 150;
const int SHOULDER_MIN = 40,  SHOULDER_MAX = 140;
const int ELBOW_MIN    = 20,  ELBOW_MAX    = 160;
const int GRIPPER_MIN  = 10,  GRIPPER_MAX  = 90;

// Bit masks for Port F (A0-A3 = PF0-PF3)
#define BASE_PIN     (1 << PF0)
#define SHOULDER_PIN (1 << PF1)
#define ELBOW_PIN    (1 << PF2)
#define GRIPPER_PIN  (1 << PF3)

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

// E-Stop — D2 = PE4 = INT4
#define ESTOP_DDR  DDRE
#define ESTOP_PORT PORTE
#define ESTOP_PINR PINE
#define ESTOP_BIT  PE4

// TCS3200 control — A4-A7 = PF4-PF7
#define TCS_CTRL_DDR  DDRF
#define TCS_CTRL_PORT PORTF
#define TCS_S0_BIT    PF4
#define TCS_S1_BIT    PF5
#define TCS_S2_BIT    PF6
#define TCS_S3_BIT    PF7

// TCS3200 OUT — D22 = PA0 (input, polled)
#define TCS_OUT_DDR  DDRA
#define TCS_OUT_PORT PORTA
#define TCS_OUT_PINR PINA
#define TCS_OUT_BIT  PA0

// =============================================================
// State
// =============================================================

volatile TState        buttonState     = STATE_RUNNING;
volatile bool          stateChanged    = false;
volatile unsigned long lastButtonIsrMs = 0;
volatile uint8_t       motorSpeed      = SPEED_DEFAULT;
volatile uint8_t       currentDir      = DIR_STOP;

// =============================================================
// E-Stop ISR — INT4 on PE4 (D2)
// =============================================================

ISR(INT4_vect) {
    unsigned long now = millis();
    if ((unsigned long)(now - lastButtonIsrMs) < 50) return;
    lastButtonIsrMs = now;

    uint8_t pressed = (ESTOP_PINR & (1 << ESTOP_BIT)) ? 0 : 1;

    if (buttonState == STATE_RUNNING && pressed) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
        stop();
    } else if (buttonState == STATE_STOPPED && !pressed) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
}

// =============================================================
// Servo PWM — Timer 4 CTC (replaces Timer 1)
// =============================================================

void updateTicks(void) {
    for (int i = 0; i < 4; i++) {
        servoTicks[i] = 2000 + (unsigned int)((long)curPos[i] * 2000 / 180);
    }
}

ISR(TIMER4_COMPA_vect) {
    PORTF |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

    bool bA = true, sA = true, eA = true, gA = true;
    while (bA || sA || eA || gA) {
        unsigned int elapsed = TCNT4;
        if (elapsed >= servoTicks[0]) { PORTF &= ~BASE_PIN;     bA = false; }
        if (elapsed >= servoTicks[1]) { PORTF &= ~SHOULDER_PIN; sA = false; }
        if (elapsed >= servoTicks[2]) { PORTF &= ~ELBOW_PIN;    eA = false; }
        if (elapsed >= servoTicks[3]) { PORTF &= ~GRIPPER_PIN;  gA = false; }
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

        case COMMAND_ARM_BASE:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[0] = constrain((int)cmd->params[0], BASE_MIN, BASE_MAX);
            break;

        case COMMAND_ARM_SHOULDER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[1] = constrain((int)cmd->params[0], SHOULDER_MIN, SHOULDER_MAX);
            break;

        case COMMAND_ARM_ELBOW:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[2] = constrain((int)cmd->params[0], ELBOW_MIN, ELBOW_MAX);
            break;

        case COMMAND_ARM_GRIPPER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            targetPos[3] = constrain((int)cmd->params[0], GRIPPER_MIN, GRIPPER_MAX);
            break;

        case COMMAND_ARM_HOME:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            for (int i = 0; i < 4; i++) targetPos[i] = 90;
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
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    // TCS3200 control — A4-A7 (PF4-PF7) as outputs
    TCS_CTRL_DDR |= (1 << TCS_S0_BIT) | (1 << TCS_S1_BIT)
                  | (1 << TCS_S2_BIT) | (1 << TCS_S3_BIT);

    // TCS3200 OUT — D22 (PA0) as input, no pull-up
    TCS_OUT_DDR  &= ~(1 << TCS_OUT_BIT);
    TCS_OUT_PORT &= ~(1 << TCS_OUT_BIT);

    // 20% scaling: S0=HIGH, S1=LOW
    TCS_CTRL_PORT |=  (1 << TCS_S0_BIT);
    TCS_CTRL_PORT &= ~(1 << TCS_S1_BIT);

    // E-Stop — D2 (PE4) input with pull-up, INT4 any-change
    ESTOP_DDR  &= ~(1 << ESTOP_BIT);
    ESTOP_PORT |=  (1 << ESTOP_BIT);
    EICRB &= ~((1 << ISC41) | (1 << ISC40));
    EICRB |=  (1 << ISC40);
    EIMSK |=  (1 << INT4);

    // Servo signal pins — A0-A3 (PF0-PF3) as outputs
    DDRF |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

    // Timer 4: 16-bit CTC, 20ms period for servo PWM
    // 16MHz / 8 = 2MHz clock → OCR4A=40000 → 20ms
    cli();
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;
    OCR4A  = 40000;
    TCCR4B |= (1 << WGM42);   // CTC mode
    TCCR4B |= (1 << CS41);    // prescaler 8
    TIMSK4 |= (1 << OCIE4A);  // enable compare-match interrupt
    sei();

    updateTicks();

    // Bare-metal motor driver — Timer 1 + Timer 3 + shift register
    motorsInit();
}

// =============================================================
// loop()
// =============================================================

void loop(void) {
    processMovement();

    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
