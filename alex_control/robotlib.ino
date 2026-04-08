/*
 * robotlib.ino
 * CG2111A — Alex Robot
 */

#define STOP_DIR 0
#define GO       1
#define BACK     2
#define CCW      3
#define CW       4

#define FL_FWD (1 << 0)
#define FL_BWD (1 << 6)
#define FR_FWD (1 << 2)
#define FR_BWD (1 << 3)
#define BL_FWD (1 << 5)
#define BL_BWD (1 << 7)
#define BR_FWD (1 << 1)
#define BR_BWD (1 << 4)

static uint8_t  _currentSr       = 0x00;
static uint8_t  _swDuty          = 255;
static uint32_t _swTimer         = 0;
#define SW_PWM_PERIOD_MS            20

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

    PORTH &= ~(1 << PH4);

    DDRB |= (1 << PB5);
    DDRE |= (1 << PE5);

    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);
    OCR1A  = 255;

    TCCR3A = (1 << COM3C1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31) | (1 << CS30);
    OCR3C  = 255;

    srWrite(0x00);
}

void updateMotorPWM(void) {
    if (_currentSr == 0x00) return;
    if (_swDuty == 255) {
        srWrite(_currentSr);
        return;
    }

    uint32_t now     = (uint32_t)millis();
    uint32_t elapsed = now - _swTimer;

    if (elapsed >= (uint32_t)SW_PWM_PERIOD_MS) {
        _swTimer = now;
        elapsed  = 0;
    }

    uint32_t onTime = (uint32_t)(SW_PWM_PERIOD_MS * _swDuty / 255);

    if (elapsed < onTime) {
        srWrite(_currentSr);
    } else {
        srWrite(0x00);
    }
}

void setMotorSpeed(uint8_t duty) {
    _swDuty  = duty;
    _swTimer = 0;
}

void move(int speed, int direction) {
    uint8_t sr = 0x00;

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
            sr = 0x00;
            break;
    }

    _currentSr = sr;
    setMotorSpeed((uint8_t)speed);
    srWrite(sr);
}

void forward(int speed)  { move(speed, GO);      }
void backward(int speed) { move(speed, BACK);     }
void ccw(int speed)      { move(speed, CCW);      }
void cw(int speed)       { move(speed, CW);       }
void stop(void)          {
    _currentSr = 0x00;
    _swDuty    = 0;
    srWrite(0x00);
}
