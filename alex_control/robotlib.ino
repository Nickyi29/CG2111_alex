/*
 * robotlib.ino
 * CG2111A — Alex Robot
 *
 * Speed control via software PWM on shift register direction bits.
 * All L293D enables are hardwired HIGH on this shield.
 * Speed is controlled by toggling direction bits ON/OFF at a duty cycle.
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

// Software PWM state
static uint8_t  _currentSr   = 0x00;  // current direction byte
static uint8_t  _swDuty      = 255;   // 0-255, 255=always on, 128=50% etc
static uint32_t _swTimer     = 0;
#define SW_PWM_PERIOD_MS      20       // 20ms period = 50Hz software PWM

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

    PORTH &= ~(1 << PH4);  // OE active LOW

    // These pins not used for speed on this shield
    // but set as outputs to avoid floating
    DDRB |= (1 << PB5);    // D11
    DDRE |= (1 << PE5);    // D3
    OCR1A = 0;
    OCR3C = 0;

    srWrite(0x00);
}

// Call this every loop() iteration to maintain software PWM
void updateMotorPWM(void) {
    if (_currentSr == 0x00) return;  // stopped — nothing to do
    if (_swDuty == 255) {
        srWrite(_currentSr);         // full speed — always on
        return;
    }

    uint32_t now     = (uint32_t)millis();
    uint32_t elapsed = now - _swTimer;

    if (elapsed >= (uint32_t)SW_PWM_PERIOD_MS) {
        _swTimer = now;
        elapsed  = 0;
    }

    // ON time within the period proportional to duty cycle
    uint32_t onTime = (uint32_t)(SW_PWM_PERIOD_MS * _swDuty / 255);

    if (elapsed < onTime) {
        srWrite(_currentSr);   // ON phase — motors running
    } else {
        srWrite(0x00);         // OFF phase — motors coasting
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
