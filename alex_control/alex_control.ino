/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 */

 //Modifed from sensor_miniproject_template.ino

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>

#include "packets.h"
#include "serial_driver.h"

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
// Pin mapping 
// =============================================================
// D2  <- E-Stop button circuit
// D3  <- TCS3200 OUT
// D22 <- TCS3200 S0
// D23 <- TCS3200 S1
// D24 <- TCS3200 S2
// D25 <- TCS3200 S3

// E-STOP on D2 = PE4 = INT4
#define ESTOP_DDR   DDRE
#define ESTOP_PORT  PORTE
#define ESTOP_PINR  PINE
#define ESTOP_BIT   PE4

// TCS3200 OUT on D3 = PE5
#define TCS_OUT_DDR   DDRE
#define TCS_OUT_PORT  PORTE
#define TCS_OUT_PINR  PINE
#define TCS_OUT_BIT   PE5

// TCS3200 S0/S1/S2/S3 on D22/D23/D24/D25 = PA0/PA1/PA2/PA3
#define TCS_CTRL_DDR   DDRA
#define TCS_CTRL_PORT  PORTA
#define TCS_S0_BIT     PA0   // D22
#define TCS_S1_BIT     PA1   // D23
#define TCS_S2_BIT     PA2   // D24
#define TCS_S3_BIT     PA3   // D25

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool stateChanged = false;
volatile unsigned long lastButtonIsrMs = 0;
volatile uint8_t motorSpeed = 150;
ISR(INT4_vect) {
    unsigned long now = millis();
    if ((unsigned long)(now - lastButtonIsrMs) < 50) return;
    lastButtonIsrMs = now;

    uint8_t pressed = (ESTOP_PINR & (1 << ESTOP_BIT)) ? 0 : 1;

    if (buttonState == STATE_RUNNING && pressed) {
        buttonState = STATE_STOPPED;
        stateChanged = true;
    } else if (buttonState == STATE_STOPPED && !pressed) {
        buttonState = STATE_RUNNING;
        stateChanged = true;
    }
}

// =============================================================
// TCS3200 helpers
// =============================================================

static inline void writeBit(volatile uint8_t &reg, uint8_t bit, bool high) {
    if (high) reg |= (1 << bit);
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

    uint32_t risingEdges = 0;
    uint8_t prev = readTcsOut();
    unsigned long startMs = millis();

    while ((unsigned long)(millis() - startMs) < 100) {
        uint8_t cur = readTcsOut();
        if (!prev && cur) risingEdges++;
        prev = cur;
    }

    return risingEdges * 10UL;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannelHz(0, 0); // red
    *g = measureChannelHz(1, 1); // green
    *b = measureChannelHz(0, 1); // blue
}

// =============================================================
// Command handler
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP: {
            cli();
            buttonState = STATE_STOPPED;
            stateChanged = false;
            sei();
            stop();

            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command = RESP_OK;
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
            pkt.command = RESP_COLOR;
            pkt.params[0] = r;
            pkt.params[1] = g;
            pkt.params[2] = b;
            strncpy(pkt.data, "Color sample ready", sizeof(pkt.data) - 1);
            pkt.data[sizeof(pkt.data) - 1] = '\0';
            sendFrame(&pkt);
            break;
        }
        case COMMAND_FORWARD:{
            forward(motorSpeed);
            break;
        }
        case COMMAND_BACKWARD:{
            backward(motorSpeed);
            break;
        }
        case COMMAND_LEFT:{
            ccw(motorSpeed);
            break;
        }
        case COMMAND_RIGHT:{
            cw(motorSpeed);
            break;
        }
        case COMMAND_SPEED: {
            int delta = (int)cmd->params[0];
            int newSpeed = motorSpeed + delta * 20;
            if (newSpeed < 0) newSpeed = 0;
            if (newSpeed > 255) newSpeed = 255;
            motorSpeed = newSpeed;
            break;
        }

        default:
            break;
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    // TCS3200 control pins: D22-D25 = PA0-PA3
    TCS_CTRL_DDR |= (1 << TCS_S0_BIT) | (1 << TCS_S1_BIT) | (1 << TCS_S2_BIT) | (1 << TCS_S3_BIT);

    // TCS3200 OUT pin: D3 = PE5 input
    TCS_OUT_DDR &= ~(1 << TCS_OUT_BIT);
    TCS_OUT_PORT &= ~(1 << TCS_OUT_BIT);

    // Required 20% scaling: S0 = HIGH, S1 = LOW
    TCS_CTRL_PORT |= (1 << TCS_S0_BIT);
    TCS_CTRL_PORT &= ~(1 << TCS_S1_BIT);

    // E-Stop input: D2 = PE4 input
    ESTOP_DDR &= ~(1 << ESTOP_BIT);
    ESTOP_PORT &= ~(1 << ESTOP_BIT);

    // INT4 on any logical change
    EICRB &= ~((1 << ISC41) | (1 << ISC40));
    EICRB |= (1 << ISC40);
    EIMSK |= (1 << INT4);

    sei();
}

void loop() {
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
