/*
 * robotlib.ino
 * CG2111A — Alex Robot
 *
 * CHANGES FROM PREVIOUS VERSION:
 *
 * 1. PWM frequency lowered — prescaler changed from 8 to 64
 *    Old: 16MHz / 8  / 256 = 7812 Hz → too high for L293D, causes ringing
 *    New: 16MHz / 64 / 256 =  976 Hz → within L293D spec, no ringing
 *
 * 2. Both L293D chips now driven from OCR1A (Timer1/D11) only
 *    OCR3C (Timer3/D3) is not connected to either L293D enable pin
 *    on this shield version — setting it had no effect
 *    OCR3C is still set alongside OCR1A in case shield is rewired
 *
 * 3. M1/M2 and M3/M4 bit assignments swapped to match physical wiring
 *    Bits 0x01-0x08 were driving left side, now correctly assigned
 *    Bits 0x10-0x80 were driving right side, now correctly assigned
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
// Shift register bit positions
// Based on observed behaviour from motor test:
// Bits 0x01-0x08 control LEFT side motors (Front Left, Back Left)
// Bits 0x10-0x80 control RIGHT side motors (Front Right, Back Right)
// =============================================================

// M1 = FRONT LEFT
#define M1_IN1 (1 << 2)   // 0x04
#define M1_IN2 (1 << 3)   // 0x08

// M2 = BACK LEFT
#define M2_IN1 (1 << 0)   // 0x01
#define M2_IN2 (1 << 1)   // 0x02

// M3 = BACK RIGHT
#define M3_IN1 (1 << 6)   // 0x40
#define M3_IN2 (1 << 7)   // 0x80

// M4 = FRONT RIGHT
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

        PORTG |= (1 << PG5);   // CLK HIGH
    }

    PORTB |= (1 << PB6);   // LATCH HIGH
}

// =============================================================
// motorsInit — call once from setup()
// =============================================================

void motorsInit(void) {
    // Shift register pins as outputs
    DDRG  |= (1 << PG5);   // D4  CLK
    DDRH  |= (1 << PH4);   // D7  OE
    DDRH  |= (1 << PH5);   // D8  DATA
    DDRB  |= (1 << PB6);   // D12 LATCH

    // OE active LOW — enable shift register outputs
    PORTH &= ~(1 << PH4);

    // PWM output pins as outputs
    DDRB  |= (1 << PB5);   // D11 OC1A — main PWM for all motors
    DDRE  |= (1 << PE5);   // D3  OC3C — secondary (may not be connected)

    // ------------------------------------------------------------------
    // Timer 1: Fast PWM 8-bit, non-inverting on OC1A (D11)
    // Prescaler 64 → frequency = 16MHz / 64 / 256 = 976 Hz
    // This is within the L293D recommended range and eliminates ringing
    // ------------------------------------------------------------------
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);  // prescaler 64
    OCR1A  = 0;

    // ------------------------------------------------------------------
    // Timer 3: Fast PWM 8-bit, non-inverting on OC3C (D3)
    // Also prescaler 64 for consistency
    // May not be connected on this shield version but set anyway
    // ------------------------------------------------------------------
    TCCR3A = (1 << COM3C1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31) | (1 << CS30);  // prescaler 64
    OCR3C  = 0;

    // All motors stopped
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
            // All wheels forward
            sr = M1_IN1 | M2_IN1 | M3_IN1 | M4_IN1;
            break;

        case BACK:
            // All wheels backward
            sr = M1_IN2 | M2_IN2 | M3_IN2 | M4_IN2;
            break;

        case CW:
            // Clockwise: left wheels forward, right wheels backward
            sr = M1_IN1 | M2_IN1 | M3_IN2 | M4_IN2;
            break;

        case CCW:
            // Counter-clockwise: right wheels forward, left wheels backward
            sr = M3_IN1 | M4_IN1 | M1_IN2 | M2_IN2;
            break;

        case STOP_DIR:
        default:
            sr  = 0x00;
            pwm = 0;
            break;
    }

    srWrite(sr);
    OCR1A = pwm;   // main PWM — drives all motors on this shield
    OCR3C = pwm;   // secondary PWM — set for completeness
}

// =============================================================
// Convenience wrappers
// =============================================================

void forward(int speed)  { move(speed, GO);      }
void backward(int speed) { move(speed, BACK);     }
void ccw(int speed)      { move(speed, CCW);      }
void cw(int speed)       { move(speed, CW);       }
void stop(void)          { move(0,     STOP_DIR); }
