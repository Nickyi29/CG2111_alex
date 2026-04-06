/*
 * robotlib.ino
 * CG2111A — Alex Robot
 *
 * Bare-metal DC motor driver.
 * testMotors() is temporarily added for bit layout calibration.
 * REMOVE testMotors() call from setup() after calibration is done.
 */

// =============================================================
// Direction constants (must match values in alex_control.ino)
// =============================================================

#define STOP_DIR 0
#define GO       1
#define BACK     2
#define CCW      3
#define CW       4

// =============================================================
// Shift register bit positions for each motor
// =============================================================

// M1 = FRONT_RIGHT (DC#1 terminal)
#define M1_IN1 (1 << 2)   // 0x04
#define M1_IN2 (1 << 3)   // 0x08

// M2 = BACK_RIGHT (DC#2 terminal)
#define M2_IN1 (1 << 0)   // 0x01
#define M2_IN2 (1 << 1)   // 0x02

// M3 = BACK_LEFT  (DC#3 terminal)
#define M3_IN1 (1 << 6)   // 0x40
#define M3_IN2 (1 << 7)   // 0x80

// M4 = FRONT_LEFT (DC#4 terminal)
#define M4_IN1 (1 << 4)   // 0x10
#define M4_IN2 (1 << 5)   // 0x20

// =============================================================
// Low-level shift register write
// =============================================================

static void srWrite(uint8_t data) {
    PORTB &= ~(1 << PB6);   // LATCH (D12) LOW

    for (int8_t i = 7; i >= 0; i--) {
        PORTG &= ~(1 << PG5);  // CLK (D4) LOW

        if (data & (1 << i))
            PORTH |=  (1 << PH5);  // DATA (D8) HIGH
        else
            PORTH &= ~(1 << PH5);  // DATA (D8) LOW

        PORTG |= (1 << PG5);   // CLK HIGH — latch bit on rising edge
    }

    PORTB |= (1 << PB6);   // LATCH HIGH
}

// =============================================================
// motorsInit — call once from setup()
// =============================================================

void motorsInit(void) {
    DDRG  |= (1 << PG5);   // D4  CLK
    DDRH  |= (1 << PH4);   // D7  OE
    DDRH  |= (1 << PH5);   // D8  DATA
    DDRB  |= (1 << PB6);   // D12 LATCH

    // OE active LOW — pull LOW to enable shift register outputs
    PORTH &= ~(1 << PH4);

    DDRB  |= (1 << PB5);   // D11 OC1A  M1/M2 speed
    DDRE  |= (1 << PE5);   // D3  OC3C  M3/M4 speed

    // Timer 1: Fast PWM 8-bit, OC1A, prescaler 8
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11);
    OCR1A  = 0;

    // Timer 3: Fast PWM 8-bit, OC3C, prescaler 8
    TCCR3A = (1 << COM3C1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31);
    OCR3C  = 0;

    srWrite(0x00);
}

// =============================================================
// testMotors — TEMPORARY, remove after calibration
// Fires each shift register bit one at a time for 2 seconds.
// Watch which wheel spins and note the direction for each bit.
//
// Record your results:
//   0x01 -> which wheel? which direction?
//   0x02 -> which wheel? which direction?
//   0x04 -> which wheel? which direction?
//   0x08 -> which wheel? which direction?
//   0x10 -> which wheel? which direction?
//   0x20 -> which wheel? which direction?
//   0x40 -> which wheel? which direction?
//   0x80 -> which wheel? which direction?
// =============================================================

void testMotors(void) {
    // Each bit is tested TWICE:
    // First with OCR1A=150, OCR3C=0   (only Timer1 PWM active)
    // Then  with OCR1A=0,   OCR3C=150 (only Timer3 PWM active)
    // This tells us which PWM channel each motor responds to

    uint8_t testBits[] = {
        0x01, 0x02, 0x04, 0x08,
        0x10, 0x20, 0x40, 0x80
    };

    delay(3000);  // startup delay

    for (int i = 0; i < 8; i++) {

        // --- Phase A: only Timer1 PWM (OCR1A) ---
        // If a wheel spins here, that motor is enabled by D11/OC1A
        OCR1A = 150;
        OCR3C = 0;
        srWrite(testBits[i]);
        delay(3000);   // watch which wheel spins
        srWrite(0x00);
        OCR1A = 0;
        OCR3C = 0;
        delay(1500);   // pause between phases

        // --- Phase B: only Timer3 PWM (OCR3C) ---
        // If a wheel spins here, that motor is enabled by D3/OC3C
        OCR1A = 0;
        OCR3C = 150;
        srWrite(testBits[i]);
        delay(3000);   // watch which wheel spins
        srWrite(0x00);
        OCR1A = 0;
        OCR3C = 0;
        delay(2000);   // longer pause between bits so you know a new round started
    }
}

// =============================================================
// move — set direction and speed for all four motors
// =============================================================

void move(int speed, int direction) {
    uint8_t sr  = 0x00;
    uint8_t pwm = (uint8_t)speed;

    switch (direction) {

        case GO:
            sr = M1_IN1 | M2_IN1 | M3_IN1 | M4_IN1;
            break;

        case BACK:
            sr = M1_IN2 | M2_IN2 | M3_IN2 | M4_IN2;
            break;

        case CW:
            // Left wheels forward, right wheels backward
            sr = M3_IN1 | M4_IN1 | M1_IN2 | M2_IN2;
            break;

        case CCW:
            // Right wheels forward, left wheels backward
            sr = M1_IN1 | M2_IN1 | M3_IN2 | M4_IN2;
            break;

        case STOP_DIR:
        default:
            sr  = 0x00;
            pwm = 0;
            break;
    }

    srWrite(sr);
    OCR1A = pwm;
    OCR3C = pwm;
}

// =============================================================
// Convenience wrappers
// =============================================================

void forward(int speed)  { move(speed, GO);      }
void backward(int speed) { move(speed, BACK);     }
void ccw(int speed)      { move(speed, CCW);      }
void cw(int speed)       { move(speed, CW);       }
void stop(void)          { move(0,     STOP_DIR); }
