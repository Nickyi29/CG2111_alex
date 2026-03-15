/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================
#define ESTOP_DDR   DDRE
#define ESTOP_PORT  PORTE
#define ESTOP_PINR  PINE       // PIN-read register for Port E (distinct from PORTE)
#define ESTOP_BIT   PE4        // bit 4 within Port E
#define DEBOUNCE_DELAY_MS 50UL
volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;
// Last accepted transition time, used for debouncing.
// Also volatile because the ISR writes it and reads it.
volatile unsigned long lastDebounceTime = 0;
//Implement the E-Stop ISR.
//RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
//STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
ISR(INT4_vect) {
    // --- Debounce: ignore edges that arrive too soon after the last one ---
    unsigned long now = millis();
    if ((unsigned long)(now - lastDebounceTime) < DEBOUNCE_DELAY_MS) {
        return;   // Likely bounce — discard this edge
    }
    lastDebounceTime = now;   // Accept this edge; update the timestamp

    // --- Read the current pin level
    // PINE is the Port E input register.  Masking with (1 << ESTOP_BIT) isolates bit 4. The result is non-zero when the pin is HIGH.
    bool pinHigh = (ESTOP_PINR & (1 << ESTOP_BIT)) != 0;

    if (pinHigh && (buttonState == STATE_RUNNING)) {
        // Valid transition: button pressed while RUNNING → go to STOPPED
        buttonState  = STATE_STOPPED;
        stateChanged = true;

    } else if (!pinHigh && (buttonState == STATE_STOPPED)) {
        // Valid transition: button released while STOPPED → go to RUNNING
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
}




// =============================================================
// Color sensor (TCS3200)
// =============================================================
//   TCS3200 pin | Arduino Mega pin | Port / bit | Description
//   ------------|------------------|------------|----------------------------
//   S0          | 22               | PA0        | Frequency scaling select
//   S1          | 23               | PA1        | Frequency scaling select
//   S2          | 24               | PA2        | Color channel select
//   S3          | 25               | PA3        | Color channel select
//   OUT         | 3  (D3)          | PE5        | Square-wave frequency output
//   VCC         | 5V               |            |
//   GND         | GND              |            |
// S0=HIGH, S1=LOW -> 20% output frequency scaling (required by handout).
// Channel selection (S2, S3):
//   Red   -> S2=LOW,  S3=LOW
//   Green -> S2=HIGH, S3=HIGH
//   Blue  -> S2=LOW,  S3=HIGH

// --- Control pins S0-S3: Port A (Arduino pins 22-25) ---
#define COLOR_DDR    DDRA
#define COLOR_PORT   PORTA

#define COLOR_S0_BIT PA0
#define COLOR_S1_BIT PA1
#define COLOR_S2_BIT PA2
#define COLOR_S3_BIT PA3

// --- OUT frequency pin: Port E, bit 5 (Arduino pin D3 / PE5) ---
// OUT is on a different port from S0-S3, so it needs its own register set.
#define COLOR_OUT_DDR   DDRE
#define COLOR_OUT_PORT  PORTE
#define COLOR_OUT_PINR  PINE   // read-register for Port E inputs
#define COLOR_OUT_BIT   PE5    // bit 5 within Port E
// After switching S2/S3 to a new channel, wait this many milliseconds
// before starting to count, so the sensor's output has time to settle.
#define COLOR_SETTLE_MS  5
// Counting window length in milliseconds.
#define COLOR_WINDOW_MS  100

static void setupColorSensor() {
    // Set S0, S1, S2, S3 as outputs.
    // DDRA bit = 1 -> output.  All four are on Port A.
    COLOR_DDR |=  (1 << COLOR_S0_BIT) |
                  (1 << COLOR_S1_BIT) |
                  (1 << COLOR_S2_BIT) |
                  (1 << COLOR_S3_BIT);

    // Set OUT (PE5 / D3) as input.
    COLOR_OUT_DDR &= ~(1 << COLOR_OUT_BIT);

    // Clearing the PORTE bit disables the internal pull-up on PE5.
    COLOR_OUT_PORT &= ~(1 << COLOR_OUT_BIT);

    // S0=HIGH, S1=LOW -> 20% output frequency scaling (required setting).
    // This stays constant; we never change S0/S1 again.
    COLOR_PORT |=  (1 << COLOR_S0_BIT);   // S0 = HIGH
    COLOR_PORT &= ~(1 << COLOR_S1_BIT);   // S1 = LOW
}


static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    // --- Select the channel by driving S2 and S3 ---
    if (s2) COLOR_PORT |=  (1 << COLOR_S2_BIT);
    else    COLOR_PORT &= ~(1 << COLOR_S2_BIT);
    if (s3) COLOR_PORT |=  (1 << COLOR_S3_BIT);
    else    COLOR_PORT &= ~(1 << COLOR_S3_BIT);

    // --- Settle: wait for the sensor output to stabilise ---
    // After a channel switch the TCS3200 needs a few ms before its OUT
    // frequency is valid.  We busy-wait using millis().
    unsigned long settleStart = millis();
    while ((millis() - settleStart) < COLOR_SETTLE_MS) { /* spin */ }

    // --- Count rising edges over the measurement window ---
    uint32_t count = 0;

    // Snapshot the initial OUT level so the first edge is detected
    // relative to whatever state the pin starts in.
    // OUT is on Port E — read PINE (COLOR_OUT_PINR), not PINA.
    bool lastHigh = (COLOR_OUT_PINR & (1 << COLOR_OUT_BIT)) != 0;

    unsigned long windowStart = millis();
    while ((millis() - windowStart) < COLOR_WINDOW_MS) {
        // Same register: OUT is on PINE (PE5), not PINA.
        bool nowHigh = (COLOR_OUT_PINR & (1 << COLOR_OUT_BIT)) != 0;
        // Rising edge = was LOW, now HIGH.
        if (!lastHigh && nowHigh) {
            count++;
        }
        lastHigh = nowHigh;
    }
    return count;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    // Each channel takes COLOR_SETTLE_MS + COLOR_WINDOW_MS ≈ 105 ms.
    // Total blocking time ≈ 315 ms — acceptable since the Pi waits for the reply.
    *r = measureChannel(0, 0) * (1000 / COLOR_WINDOW_MS);  // Red
    *g = measureChannel(1, 1) * (1000 / COLOR_WINDOW_MS);  // Green
    *b = measureChannel(0, 1) * (1000 / COLOR_WINDOW_MS);  // Blue
}

// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;
        case COMMAND_COLOR: {
            // --- Measure the three colour channels ---
            // readColorChannels() blocks for ~315 ms while it measures
            // Red, Green, and Blue in sequence.  The Pi waits patiently
            // because it sent the command and expects a reply.
            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);

            // --- Build and send the RESP_COLOR response packet ---
            // params[0] = red   frequency in Hz
            // params[1] = green frequency in Hz
            // params[2] = blue  frequency in Hz
            // All other fields are zero-initialised by memset().
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            pkt.params[0]  = r;
            pkt.params[1]  = g;
            pkt.params[2]  = b;
            sendFrame(&pkt);
            break;
        }
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.
    ESTOP_DDR  &= ~(1 << ESTOP_BIT);   // PE4 = input
    ESTOP_PORT &= ~(1 << ESTOP_BIT);
    EICRB &= ~(1 << ISC41);  // ISC41 = 0
    EICRB |=  (1 << ISC40);  // ISC40 = 1  ->  any logical change on INT4
    EIMSK |= (1 << INT4);
    // Configures the five Port A pins: S0/S1/S2/S3 as outputs and OUT
    // as input.  Also locks in S0=HIGH, S1=LOW for 20% frequency scaling.
    setupColorSensor();
    sei();
}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
