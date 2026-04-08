/*
 * robotlib.ino
 * CG2111A — Alex Robot
 *
 * CRITICAL FIX: MH Electronics shield uses D9/D10 for motor enable pins.
 * Shield designed for Uno where D9=OC1A, D10=OC1B.
 * On Mega: D9=PH6=OC2B, D10=PB4=OC2A — both on Timer2.
 * Previous code used D11(OC1A) and D3(OC3C) — wrong pins, enables floated HIGH.
 * Now using Timer2 on D9/D10 — speed control works on ALL motors.
 */

#define STOP_DIR 0
#define GO       1
#define BACK     2
#define CCW      3
#define CW       4

// FRONT LEFT  = M4 shield terminal
#define FL_FWD (1 << 0)   // 0x01
#define FL_BWD (1 << 6)   // 0x40

// FRONT RIGHT = M1 shield terminal
#define FR_FWD (1 << 2)   // 0x04
#define FR_BWD (1 << 3)   // 0x08

// BACK LEFT   = M3 shield terminal (mounted in reverse)
#define BL_FWD (1 << 5)   // 0x20
#define BL_BWD (1 << 7)   // 0x80

// BACK RIGHT  = M2 shield terminal (mounted in reverse)
#define BR_FWD (1 << 1)   // 0x02
#define BR_BWD (1 << 4)   // 0x10

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
    // Shift register control pins
    DDRG |= (1 << PG5);    // D4  CLK
    DDRH |= (1 << PH4);    // D7  OE
    DDRH |= (1 << PH5);    // D8  DATA
    DDRB |= (1 << PB6);    // D12 LATCH

    // OE active LOW — enable shift register outputs
    PORTH &= ~(1 << PH4);

    // Motor enable pins — D9 and D10 on MH Electronics shield
    // On Mega: D9 = PH6 = OC2B, D10 = PB4 = OC2A
    DDRH |= (1 << PH6);    // D9  OC2B — enable for M1/M2
    DDRB |= (1 << PB4);    // D10 OC2A — enable for M3/M4

    // Timer2: Fast PWM 8-bit, non-inverting on OC2A and OC2B
    // prescaler 64 -> 16MHz / 64 / 256 = 976Hz
    TCCR2A = (1 << COM2A1) | (1 << COM2B1)
           | (1 << WGM21)  | (1 << WGM20);
    TCCR2B = (1 << CS22);   // prescaler 64
    OCR2A  = 0;              // D10 — M3/M4 enable, start at 0
    OCR2B  = 0;              // D9  — M1/M2 enable, start at 0

    srWrite(0x00);
}

void move(int speed, int direction) {
    uint8_t sr  = 0x00;
    uint8_t pwm = (uint8_t)speed;

    switch (direction) {
        case GO:
            sr = FL_FWD | FR_FWD | BL_BWD | BR_BWD;
            break;
        case BACK:
            sr = FL_BWD | FR_BWD | BL_FWD | BR_FWD;
            break;
        case CW:
            sr = FL_BWD | FR_FWD | BL_FWD | BR_BWD;
            break;
        case CCW:
            sr = FL_FWD | FR_BWD | BL_BWD | BR_FWD;
            break;
        case STOP_DIR:
        default:
            sr  = 0x00;
            pwm = 0;
            break;
    }

    srWrite(sr);
    OCR2A = pwm;    // D10 — controls M3/M4 (left side)
    OCR2B = pwm;    // D9  — controls M1/M2 (right side)
}

void forward(int speed)  { move(speed, GO);      }
void backward(int speed) { move(speed, BACK);     }
void ccw(int speed)      { move(speed, CCW);      }
void cw(int speed)       { move(speed, CW);       }
void stop(void)          { move(0,     STOP_DIR); }
