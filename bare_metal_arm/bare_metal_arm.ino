/* CG2111A Mini-Project 1: Activity 4 - Bare-Metal Robot Arm */

// --- SAFETY LIMITS (From Activity 3) ---
const int BASE_MIN = 30,     BASE_MAX = 150;
const int SHOULDER_MIN = 40, SHOULDER_MAX = 140;
const int ELBOW_MIN = 20,    ELBOW_MAX = 160;
const int GRIPPER_MIN = 10,  GRIPPER_MAX = 90;

// Bare-Metal Pin Definitions for A0-A3 (PC0-PC3)
#define BASE_PIN     (1 << 0) 
#define SHOULDER_PIN (1 << 1) 
#define ELBOW_PIN    (1 << 2) 
#define GRIPPER_PIN  (1 << 3)

// State Variables
volatile int curPos[4] = {90, 90, 90, 90}; 
int targetPos[4] = {90, 90, 90, 90};
int msPerDeg = 10;
unsigned long lastMoveTime[4] = {0, 0, 0, 0};

// PWM Timing (Prescaler 8: 1 tick = 0.5us)
// 1000us = 2000 ticks | 1500us = 3000 ticks | 2000us = 4000 ticks
volatile unsigned int servoTicks[4];

void setup() {
  Serial.begin(115200);
  DDRC |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

  // --- TIMER 1 SETUP (16-bit Bare-Metal) ---
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  // Set OCR1A for exactly 20ms period (16MHz / 8 prescaler / 50Hz = 40,000)
  OCR1A = 40000;

  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS11);   // Prescaler 8 
  TIMSK1 |= (1 << OCIE1A); // Enable Compare Match A Interrupt
  sei();

  updateTicks();
}

// ISR triggers exactly every 20ms to generate PWM pulses
ISR(TIMER1_COMPA_vect) {
  // PWM Start: Turn ALL servo pins HIGH
  PORTC |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

  // Pulse Width Control: Turn pins LOW at specific tick counts
  bool bActive = true, sActive = true, eActive = true, gActive = true;
  while(bActive || sActive || eActive || gActive) {
    unsigned int elapsed = TCNT1;
    if (elapsed >= servoTicks[0]) { PORTC &= ~BASE_PIN; bActive = false; }
    if (elapsed >= servoTicks[1]) { PORTC &= ~SHOULDER_PIN; sActive = false; }
    if (elapsed >= servoTicks[2]) { PORTC &= ~ELBOW_PIN; eActive = false; }
    if (elapsed >= servoTicks[3]) { PORTC &= ~GRIPPER_PIN; gActive = false; }
  }
}

void updateTicks() {
  for (int i = 0; i < 4; i++) {
    // Map 0-180deg to 2000-4000 ticks (1000us-2000us)
    servoTicks[i] = 2000 + (unsigned int)((long)curPos[i] * 2000 / 180);
  }
}

// Reverted movement logic: Uses millis() for non-blocking concurrent movement
void processMovement() {
  unsigned long now = millis(); 
  for (int i = 0; i < 4; i++) {
    if (curPos[i] != targetPos[i] && (now - lastMoveTime[i] >= (unsigned long)msPerDeg)) {
      if (targetPos[i] > curPos[i]) curPos[i]++;
      else curPos[i]--;
      lastMoveTime[i] = now;
      updateTicks(); // Re-calculate ticks after movement
    }
  }
}

int parse3(String s) {
  if (s.length() != 3) return -1;
  for(int i = 0; i < 3; i++) {
    if(!isDigit(s.charAt(i))) return -1;
  }
  return s.toInt();
}

void loop() {
  processMovement();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (!cmd.length()) return;

    if (cmd == "H") {
      for (int i = 0; i < 4; i++) targetPos[i] = 90;
      return;
    }

    if (cmd.length() != 4) return;
    char c = cmd.charAt(0);
    int val = parse3(cmd.substring(1));
    if (val < 0) return;

    // Apply safety limits and set targets
    if (c == 'V') msPerDeg = val;
    else if (c == 'B') targetPos[0] = constrain(val, BASE_MIN, BASE_MAX);
    else if (c == 'S') targetPos[1] = constrain(val, SHOULDER_MIN, SHOULDER_MAX);
    else if (c == 'E') targetPos[2] = constrain(val, ELBOW_MIN, ELBOW_MAX);
    else if (c == 'G') targetPos[3] = constrain(val, GRIPPER_MIN, GRIPPER_MAX);
  }
}
