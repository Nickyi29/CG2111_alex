#include <AFMotor.h>

typedef enum {
  STOP = 0,
  GO,
  BACK,
  CCW,
  CW
} dir;

// Motor mapping from your current file
#define FRONT_LEFT   4   // M4
#define FRONT_RIGHT  1   // M1
#define BACK_LEFT    3   // M3
#define BACK_RIGHT   2   // M2

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

static void setAllSpeeds(uint8_t speed) {
  speed = constrain(speed, 0, 255);
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);
}

void move(uint8_t speed, dir direction) {
  setAllSpeeds(speed);

  switch (direction) {
    case GO:
      motorFL.run(FORWARD);
      motorFR.run(FORWARD);
      motorBL.run(BACKWARD);
      motorBR.run(BACKWARD);
      break;

    case BACK:
      motorFL.run(BACKWARD);
      motorFR.run(BACKWARD);
      motorBL.run(FORWARD);
      motorBR.run(FORWARD);
      break;

    case CW:
      motorFL.run(BACKWARD);
      motorFR.run(FORWARD);
      motorBL.run(FORWARD);
      motorBR.run(BACKWARD);
      break;

    case CCW:
      motorFL.run(FORWARD);
      motorFR.run(BACKWARD);
      motorBL.run(BACKWARD);
      motorBR.run(FORWARD);
      break;

    case STOP:
    default:
      motorFL.run(RELEASE);
      motorFR.run(RELEASE);
      motorBL.run(RELEASE);
      motorBR.run(RELEASE);
      break;
  }
}

void forward(uint8_t speed) {
  move(speed, GO);
}

void backward(uint8_t speed) {
  move(speed, BACK);
}

void ccw(uint8_t speed) {
  move(speed, CCW);
}

void cw(uint8_t speed) {
  move(speed, CW);
}

void stop() {
  move(0, STOP);
}
