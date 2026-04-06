/*
 * robotlib.ino
 * CG2111A — Alex Robot
 *
 * Bare-metal DC motor driver derived directly from the original AFMotor code.
 *
 * Physical motor mapping (from original AFMotor code):
 *   M1 shield terminal = FRONT RIGHT
 *   M2 shield terminal = BACK RIGHT
 *   M3 shield terminal = BACK LEFT
 *   M4 shield terminal = FRONT LEFT
 *
 * AFMotor 74HC595 bit assignments:
 *   M1: IN1=0x04, IN2=0x08  FORWARD=0x08, BACKWARD=0x04
 *   M2: IN1=0x02, IN2=0x01  FORWARD=0x01, BACKWARD=0x02
 *   M3: IN1=0x40, IN2=0x80  FORWARD=0x40, BACKWARD=0x80
 *   M4: IN1=0x20, IN2=0x10  FORWARD=0x10, BACKWARD=0x20
 *
 * IMPORTANT — back motors are mounted in reverse on the chassis:
 *   Robot FORWARD = FL(M4) FWD + FR(M1) FWD + BL(M3) BWD + BR(M2) BWD
 *
 * Shift register byte values per direction:
 *   GO   = 0x10|0x08|0x80|0x02 = 0x9A
 *   BACK = 0x20|0x04|0x40|0x01 = 0x65
 *   CW   = 0x20|0x08|0x40|0x02 = 0x6A
 *   CCW  = 0x10|0x04|0x80|0x01 = 0x95
 */

#define STOP_DIR 0
#define GO       1
#define BACK     2
#define CCW      3
#define CW       4

// FRONT LEFT  = M4 shield terminal  ← UNCHANGED
#define FL_FWD (1 << 4)   // 0x10
#define FL_BWD (1 << 5)   // 0x20

// FRONT RIGHT = M1 shield terminal  ← FWD/BWD SWAPPED
#define FR_FWD (1 << 2)   // 0x04  (was 1<<3)
#define FR_BWD (1 << 3)   // 0x08  (was 1<<2)

// BACK LEFT   = M3 shield terminal  ← FWD/BWD SWAPPED
#define BL_FWD (1 << 7)   // 0x80  (was 1<<6)
#define BL_BWD (1 << 6)   // 0x40  (was 1<<7)

// BACK RIGHT  = M2 shield terminal  ← UNCHANGED
#define BR_FWD (1 << 0)   // 0x01
#define BR_BWD (1 << 1)   // 0x02

static void srWrite(uint8_t data) {
    PORTB &= ~(1 << PB6);

    for (int8_t i = 7; i >= 0; i--) {
        PORTG &= ~(1 << PG5);

        if (data & (1 << i))
            PORTH |=  (1 << PH5);
        else
            PORTH &= ~(1 << PH5);

        PORTG |= (1 << PG5);
    }

    PORTB |= (1 << PB6);
}

void motorsInit(void) {
    DDRG |= (1 << PG5);
    DDRH |= (1 << PH4);
    DDRH |= (1 << PH5);
    DDRB |= (1 << PB6);

    PORTH &= ~(1 << PH4);  // OE active LOW — enable outputs

    DDRB |= (1 << PB5);    // D11 OC1A
    DDRE |= (1 << PE5);    // D3  OC3C

    // Timer 1: Fast PWM 8-bit, prescaler 64 -> 976 Hz
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);
    OCR1A  = 0;

    // Timer 3: Fast PWM 8-bit, prescaler 64
    TCCR3A = (1 << COM3C1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31) | (1 << CS30);
    OCR3C  = 0;

    srWrite(0x00);
}

void move(int speed, int direction) {
    uint8_t sr  = 0x00;
    uint8_t pwm = (uint8_t)speed;

    switch (direction) {
        case GO:
            // FL FWD + FR FWD + BL BWD + BR BWD = 0x9A
            sr = FL_FWD | FR_FWD | BL_BWD | BR_BWD;
            break;
        case BACK:
            // FL BWD + FR BWD + BL FWD + BR FWD = 0x65
            sr = FL_BWD | FR_BWD | BL_FWD | BR_FWD;
            break;
        case CW:
            // Right turn: left wheels fwd, right wheels bwd = 0x6A
            sr = FL_BWD | FR_FWD | BL_FWD | BR_BWD;
            break;
        case CCW:
            // Left turn: right wheels fwd, left wheels bwd = 0x95
            sr = FL_FWD | FR_BWD | BL_BWD | BR_FWD;
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

void forward(int speed)  { move(speed, GO);      }
void backward(int speed) { move(speed, BACK);     }
void ccw(int speed)      { move(speed, CCW);      }
void cw(int speed)       { move(speed, CW);       }
void stop(void)          { move(0,     STOP_DIR); }
