/*
 * serial_driver.h
 * CG2111A — Alex Robot
 *
 * CHANGES FOR BARE METAL:
 *   USE_BAREMETAL_SERIAL = 1
 *   Fixed baud rate: UBRR=16 with U2X0=1 -> 117647 baud (2.1% error, within AVR tolerance)
 *   TX buffer increased from 128 to 256 bytes to hold 2 full frames safely
 */

#pragma once

#include <Arduino.h>
#include <avr/interrupt.h>
#include <string.h>
#include "packets.h"

#define USE_BAREMETAL_SERIAL 1   // ← changed from 0

#if USE_BAREMETAL_SERIAL

#define TX_BUFFER_SIZE 256       // ← increased from 128, holds 2 full frames
#define TX_BUFFER_MASK (TX_BUFFER_SIZE - 1)
#define RX_BUFFER_SIZE 256
#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1)

volatile uint8_t tx_buf[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0, tx_tail = 0;

volatile uint8_t rx_buf[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

static inline uint8_t txUsed(void) {
    return (uint8_t)((tx_head - tx_tail) & TX_BUFFER_MASK);
}

static inline uint8_t rxUsed(void) {
    return (uint8_t)((rx_head - rx_tail) & RX_BUFFER_MASK);
}

void usartInit(uint16_t ubrr) {
    // Use double-speed mode (U2X0=1) for better accuracy at 115200 baud
    // With U2X0=1 and UBRR=16: baud = 16MHz / (8 * 17) = 117647 (2.1% error)
    // This is within AVR's ±2.5% tolerance
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0A = (1 << U2X0);                              // ← double speed mode
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);            // 8-bit, no parity, 1 stop
}

bool txEnqueue(const uint8_t *data, uint8_t len) {
    uint8_t used       = txUsed();
    uint8_t free_space = (TX_BUFFER_SIZE - 1) - used;
    if (len > free_space) return false;

    uint8_t head = tx_head;
    for (uint8_t i = 0; i < len; i++) {
        tx_buf[head] = data[i];
        head = (head + 1) & TX_BUFFER_MASK;
    }
    tx_head = head;
    UCSR0B |= (1 << UDRIE0);   // enable TX empty interrupt
    return true;
}

ISR(USART0_UDRE_vect) {
    if (tx_head == tx_tail) {
        UCSR0B &= ~(1 << UDRIE0);  // buffer empty — disable interrupt
        return;
    }
    UDR0    = tx_buf[tx_tail];
    tx_tail = (tx_tail + 1) & TX_BUFFER_MASK;
}

bool rxDequeue(uint8_t *data, uint8_t len) {
    uint8_t available = rxUsed();
    if (len > available) return false;

    uint8_t tail = rx_tail;
    for (uint8_t i = 0; i < len; i++) {
        data[i] = rx_buf[tail];
        tail = (tail + 1) & RX_BUFFER_MASK;
    }
    rx_tail = tail;
    return true;
}

ISR(USART0_RX_vect) {
    uint8_t byte = UDR0;
    uint8_t next = (rx_head + 1) & RX_BUFFER_MASK;
    if (next != rx_tail) {   // only store if buffer not full
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
    // if full, byte is silently dropped — RX buffer is 256 bytes so this is rare
}

#endif  // USE_BAREMETAL_SERIAL

static uint8_t computeChecksum(const uint8_t *data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

static void sendFrame(const TPacket *pkt) {
    uint8_t frame[FRAME_SIZE];
    frame[0] = MAGIC_HI;
    frame[1] = MAGIC_LO;
    memcpy(&frame[2], pkt, TPACKET_SIZE);
    frame[2 + TPACKET_SIZE] = computeChecksum((const uint8_t *)pkt, TPACKET_SIZE);

#if USE_BAREMETAL_SERIAL
    while (!txEnqueue(frame, FRAME_SIZE)) { }  // busy wait if buffer full
#else
    Serial.write(frame, FRAME_SIZE);
#endif
}

static bool receiveFrame(TPacket *pkt) {
#if USE_BAREMETAL_SERIAL
    while (((rx_head - rx_tail) & RX_BUFFER_MASK) >= FRAME_SIZE) {
        uint8_t hi = rx_buf[rx_tail];
        uint8_t lo = rx_buf[(rx_tail + 1) & RX_BUFFER_MASK];

        if (hi == MAGIC_HI && lo == MAGIC_LO) {
            uint8_t frame[FRAME_SIZE];
            for (uint8_t i = 0; i < FRAME_SIZE; i++)
                frame[i] = rx_buf[(rx_tail + i) & RX_BUFFER_MASK];

            uint8_t expected = computeChecksum(&frame[2], TPACKET_SIZE);
            if (frame[FRAME_SIZE - 1] == expected) {
                memcpy(pkt, &frame[2], TPACKET_SIZE);
                rx_tail = (rx_tail + FRAME_SIZE) & RX_BUFFER_MASK;
                return true;
            }
        }
        // Magic bytes not found or checksum failed — advance one byte
        rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;
    }
    return false;

#else
    static uint8_t state = 0;
    static uint8_t raw[TPACKET_SIZE];
    static uint8_t index = 0;

    while (Serial.available() > 0) {
        uint8_t byte = (uint8_t)Serial.read();

        switch (state) {
            case 0:
                if (byte == MAGIC_HI) state = 1;
                break;

            case 1:
                if      (byte == MAGIC_LO) { state = 2; index = 0; }
                else if (byte != MAGIC_HI) { state = 0; }
                break;

            case 2:
                raw[index++] = byte;
                if (index >= TPACKET_SIZE) state = 3;
                break;

            case 3: {
                uint8_t expected = computeChecksum(raw, TPACKET_SIZE);
                if (byte == expected) {
                    memcpy(pkt, raw, TPACKET_SIZE);
                    state = 0;
                    return true;
                }
                state = (byte == MAGIC_HI) ? 1 : 0;
                break;
            }
        }
    }
    return false;
#endif
}
