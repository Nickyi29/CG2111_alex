/*
 * alex_control.ino
 * Studio 13: Sensor Mini-Project
 */

#include <AFMotor.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>

#include "packets.h"
#include "serial_driver.h"

// Arm state — from Mini-Project 1
const int BASE_MIN = 30,     BASE_MAX = 150;
const int SHOULDER_MIN = 40, SHOULDER_MAX = 140;
const int ELBOW_MIN = 20,    ELBOW_MAX = 160;
const int GRIPPER_MIN = 10,  GRIPPER_MAX = 90;

#define BASE_PIN     (1 << 0)
#define SHOULDER_PIN (1 << 1)
#define ELBOW_PIN    (1 << 2)
#define GRIPPER_PIN  (1 << 3)

volatile int curPos[4]          = {90, 90, 90, 90};
int          targetPos[4]       = {90, 90, 90, 90};
int          msPerDeg           = 10;
unsigned long lastMoveTime[4]   = {0, 0, 0, 0};
volatile unsigned int servoTicks[4];

// Direction constants — must match dir enum values in robotlib.ino
// Defined here as #define so they are available before robotlib.ino is merged
#define DIR_STOP 0
#define DIR_GO   1
#define DIR_BACK 2
#define DIR_CCW  3
#define DIR_CW   4

// Speed limits
#define SPEED_MIN 30    // FIX: lowered from 50 → allows slower crawl (3 steps down from 150)
#define SPEED_MAX 255
#define SPEED_DEFAULT 150
#define SPEED_STEP    50

// =============================================================
// Pin mapping
// =============================================================

#define ESTOP_DDR  DDRE
#define ESTOP_PORT PORTE
#define ESTOP_PINR PINE
#define ESTOP_BIT  PE4

#define TCS_OUT_DDR  DDRE
#define TCS_OUT_PORT PORTE
#define TCS_OUT_PINR PINE
#define TCS_OUT_BIT  PE5

#define TCS_CTRL_DDR  DDRA
#define TCS_CTRL_PORT PORTA
#define TCS_S0_BIT PA0
#define TCS_S1_BIT PA1
#define TCS_S2_BIT PA2
#define TCS_S3_BIT PA3

// =============================================================
// State
// =============================================================

volatile TState        buttonState     = STATE_RUNNING;
volatile bool          stateChanged    = false;
volatile unsigned long lastButtonIsrMs = 0;
volatile uint8_t       motorSpeed      = SPEED_DEFAULT;
volatile uint8_t       currentDir      = DIR_STOP;

ISR(INT4_vect) {
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

static inline void writeBit(volatile uint8_t &reg, uint8_t bit, bool high) {
    if (high) reg |=  (1 << bit);
    else      reg &= ~(1 << bit);
}

static inline uint8_t readTcsOut(void) {
    return (TCS_OUT_PINR & (1 << TCS_OUT_BIT)) ? 1 : 0;
}

static void setTcsFilter(uint8_t s2High, uint8_t s3High) {
    writeBit(TCS_CTRL_PORT, TCS_S2_BIT, s2High);
    writeBit(TCS_CTRL_PORT, TCS_S3_BIT, s3High);
}

static uint32_t measureChannelHz(uint8_t s2High, uint8_t s3High) {
    setTcsFilter(s2High, s3High);
    delayMicroseconds(300);

    uint32_t risingEdges  = 0;
    uint8_t  prev         = readTcsOut();
    unsigned long startMs = millis();

    while ((unsigned long)(millis() - startMs) < 100) {
        uint8_t cur = readTcsOut();
        if (!prev && cur) risingEdges++;
        prev = cur;
    }
    return risingEdges * 10UL;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannelHz(0, 0);  // red
    *g = measureChannelHz(1, 1);  // green
    *b = measureChannelHz(0, 1);  // blue
}

void updateTicks() {
    for (int i = 0; i < 4; i++) {
        servoTicks[i] = 2000 + (unsigned int)((long)curPos[i] * 2000 / 180);
    }
}

void processMovement() {
    unsigned long now = millis();
    for (int i = 0; i < 4; i++) {
        if (curPos[i] != targetPos[i] &&
            (now - lastMoveTime[i] >= (unsigned long)msPerDeg)) {
            if (targetPos[i] > curPos[i]) curPos[i]++;
            else curPos[i]--;
            lastMoveTime[i] = now;
            updateTicks();
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    PORTC |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
    bool bActive = true, sActive = true, eActive = true, gActive = true;
    while (bActive || sActive || eActive || gActive) {
        unsigned int elapsed = TCNT1;
        if (elapsed >= servoTicks[0]) { PORTC &= ~BASE_PIN;     bActive = false; }
        if (elapsed >= servoTicks[1]) { PORTC &= ~SHOULDER_PIN; sActive = false; }
        if (elapsed >= servoTicks[2]) { PORTC &= ~ELBOW_PIN;    eActive = false; }
        if (elapsed >= servoTicks[3]) { PORTC &= ~GRIPPER_PIN;  gActive = false; }
    }
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

        case COMMAND_FORWARD: {
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_GO;
            forward(motorSpeed);
            break;
        }

        case COMMAND_BACKWARD: {
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_BACK;
            backward(motorSpeed);
            break;
        }

        case COMMAND_LEFT: {
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_CCW;
            ccw(motorSpeed);
            break;
        }

        case COMMAND_RIGHT: {
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            currentDir = DIR_CW;
            cw(motorSpeed);
            break;
        }

        case COMMAND_SPEED: {
            int delta    = (cmd->params[0] == 1) ? SPEED_STEP : -SPEED_STEP;
            int newSpeed = (int)motorSpeed + delta;
            if (newSpeed < SPEED_MIN) newSpeed = SPEED_MIN;  // FIX: floor = 30
            if (newSpeed > SPEED_MAX) newSpeed = SPEED_MAX;
            motorSpeed = (uint8_t)newSpeed;
            // immediately re-apply if robot is currently moving
            if (buttonState != STATE_STOPPED && currentDir != DIR_STOP) {
                move(motorSpeed, currentDir);
            }
            // send new speed back so Pi can display it
            sendResponse(RESP_OK, motorSpeed);
            break;
        }
        case COMMAND_ARM_BASE:
        if (buttonState == STATE_STOPPED) {
            sendStatus(STATE_STOPPED);
            break;
        }
        targetPos[0] = constrain((int)cmd->params[0], BASE_MIN, BASE_MAX);
        break;

        case COMMAND_ARM_SHOULDER:
        if (buttonState == STATE_STOPPED) {
            sendStatus(STATE_STOPPED); break;
        }
        targetPos[1] = constrain((int)cmd->params[0], SHOULDER_MIN, SHOULDER_MAX);
        break;

        case COMMAND_ARM_ELBOW:
        if (buttonState == STATE_STOPPED) {
            sendStatus(STATE_STOPPED); break;
        }
        targetPos[2] = constrain((int)cmd->params[0], ELBOW_MIN, ELBOW_MAX);
        break;

        case COMMAND_ARM_GRIPPER:
        if (buttonState == STATE_STOPPED) {
            sendStatus(STATE_STOPPED); break;
        }
        targetPos[3] = constrain((int)cmd->params[0], GRIPPER_MIN, GRIPPER_MAX);
        break;
        
        case COMMAND_ARM_HOME:
        if (buttonState == STATE_STOPPED) {
            sendStatus(STATE_STOPPED); break;
        }
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
// setup() and loop()
// =============================================================

void setup() {
#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    TCS_CTRL_DDR |= (1 << TCS_S0_BIT) | (1 << TCS_S1_BIT)
                  | (1 << TCS_S2_BIT) | (1 << TCS_S3_BIT);

    TCS_OUT_DDR  &= ~(1 << TCS_OUT_BIT);
    TCS_OUT_PORT &= ~(1 << TCS_OUT_BIT);

    TCS_CTRL_PORT |=  (1 << TCS_S0_BIT);
    TCS_CTRL_PORT &= ~(1 << TCS_S1_BIT);

    // E-Stop — pull-up enabled, prevents floating pin phantom triggers
    ESTOP_DDR  &= ~(1 << ESTOP_BIT);
    ESTOP_PORT |=  (1 << ESTOP_BIT);

    // INT4 on any logical change
    EICRB &= ~((1 << ISC41) | (1 << ISC40));
    EICRB |=  (1 << ISC40);
    EIMSK |=  (1 << INT4);

    sei();
    // Arm pin setup
    DDRC |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
    // Timer 1 for servo PWM
    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
    OCR1A = 40000;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);

updateTicks();
}

void loop() {
    // arm movement — runs every iteration
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