/*
 * robotlib.ino
 * CG2111A — Alex Robot
 *
 * Bare-metal DC motor driver replacing AFMotor.
 *
 * CHANGES FROM ORIGINAL:
 *   - Removed #include <AFMotor.h> — fully bare-metal
 *   - Motor shield v1 shift register driven directly via registers
 *   - Timer 1 Fast PWM 8-bit → OC1A (D11/PB5) controls M1 + M2 speed
 *   - Timer 3 Fast PWM 8-bit → OC3C (D3/PE5)  controls M3 + M4 speed
 *
 * Pin mapping (motor shield v1, fixed by PCB traces — cannot change):
 *   D4  PG5  shift register CLOCK
 *   D7  PH4  shift register OE    (active LOW — must be driven LOW)
 *   D8  PH5  shift register DATA
 *   D12 PB6  shift register LATCH
 *   D11 PB5  OC1A  PWM for M1 + M2
 *   D3  PE5  OC3C  PWM for M3 + M4
 *
 * Motor-to-shield terminal wiring (from robotlib.ino comments):
 *   M1 → FRONT_RIGHT
 *   M2 → BACK_RIGHT
 *   M3 → BACK_LEFT
 *   M4 → FRONT_LEFT
 *
 * Shift register bit layout (74HC595, Motor Shield v1):
 *   Bit 0 (0x01) M2 IN1
 *   Bit 1 (0x02) M2 IN2
 *   Bit 2 (0x04) M1 IN1
 *   Bit 3 (0x08) M1 IN2
 *   Bit 4 (0x10) M4 IN1   (FRONT_LEFT)
 *   Bit 5 (0x20) M4 IN2
 *   Bit 6 (0x40) M3 IN1   (BACK_LEFT)
 *   Bit 7 (0x80) M3 IN2
 *
 * For each motor, FORWARD = IN1 HIGH / IN2 LOW,
 *                 BACKWARD = IN1 LOW  / IN2 HIGH.
 * If a motor runs the wrong way, swap IN1/IN2 bits for that motor.
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
    // Pull latch LOW before clocking data
    PORTB &= ~(1 << PB6);   // LATCH (D12) LOW

    // Shift out 8 bits, MSB first
    for (int8_t i = 7; i >= 0; i--) {
        PORTG &= ~(1 << PG5);  // CLK (D4) LOW

        if (data & (1 << i))
            PORTH |=  (1 << PH5);  // DATA (D8) HIGH
        else
            PORTH &= ~(1 << PH5);  // DATA (D8) LOW

        PORTG |= (1 << PG5);   // CLK (D4) HIGH — latch bit on rising edge
    }

    // Latch data to outputs: LATCH HIGH
    PORTB |= (1 << PB6);
}

// =============================================================
// motorsInit — call once from setup()
// =============================================================

void motorsInit(void) {
    // Shift register control pins as outputs
    DDRG  |= (1 << PG5);   // D4  CLK
    DDRH  |= (1 << PH4);   // D7  OE
    DDRH  |= (1 << PH5);   // D8  DATA
    DDRB  |= (1 << PB6);   // D12 LATCH

    // OE is active LOW on the 74HC595; pull it LOW to enable outputs
    PORTH &= ~(1 << PH4);

    // PWM output pins as outputs
    DDRB  |= (1 << PB5);   // D11 OC1A  M1/M2 speed
    DDRE  |= (1 << PE5);   // D3  OC3C  M3/M4 speed

    // ------------------------------------------------------------------
    // Timer 1: Fast PWM 8-bit, non-inverting on OC1A (D11), prescaler 8
    // Frequency = 16MHz / 8 / 256 ≈ 7.8 kHz
    // ------------------------------------------------------------------
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11);
    OCR1A  = 0;

    // ------------------------------------------------------------------
    // Timer 3: Fast PWM 8-bit, non-inverting on OC3C (D3), prescaler 8
    // ------------------------------------------------------------------
    TCCR3A = (1 << COM3C1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31);
    OCR3C  = 0;

    // Clear shift register — all motors stopped
    srWrite(0x00);
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
    OCR1A = pwm;   // M1/M2 speed
    OCR3C = pwm;   // M3/M4 speed
}

// =============================================================
// Convenience wrappers
// =============================================================

void forward(int speed)  { move(speed, GO);      }
void backward(int speed) { move(speed, BACK);     }
void ccw(int speed)      { move(speed, CCW);      }
void cw(int speed)       { move(speed, CW);       }
void stop(void)          { move(0,     STOP_DIR); }
