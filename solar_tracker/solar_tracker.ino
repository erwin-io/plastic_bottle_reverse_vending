#include <Servo.h>

/* ================== SERVO SETUP ================== */
Servo panServo;    // LEFT-RIGHT (spin)
Servo tiltServo;   // UP-DOWN (tilt)

const int PAN_PIN  = 9;
const int TILT_PIN = 10;

/* ================== LDR PINS ================== */
const int LDR_TOP    = A0;
const int LDR_RIGHT  = A1;
const int LDR_BOTTOM = A2;
const int LDR_LEFT   = A3;

/* ================== START POSITIONS ================== */
int panAngle  = 90;
int tiltAngle = 90;

/* ================== SERVO LIMITS ================== */
// Adjust if your frame blocks movement
const int PAN_MIN  = 5;
const int PAN_MAX  = 175;

const int TILT_MIN = 25;
const int TILT_MAX = 160;

/* ================== TUNING ================== */
// Increase DEAD_BAND if shaking
// Increase STEP if too slow
const int SAMPLE_N  = 8;     // averaging
const int DEAD_BAND = 40;    // try 50-70 if jitter
const int STEP      = 1;     // try 2 if slow
const int LOOP_MS   = 20;

/* ================== FUNCTIONS ================== */
int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLE_N; i++) {
    sum += analogRead(pin);
    delayMicroseconds(600);
  }
  return (int)(sum / SAMPLE_N);
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  delay(800);
  Serial.println("Sun Tracker Started");
}

/* ================== LOOP ================== */
void loop() {

  int top    = readAvg(LDR_TOP);
  int right  = readAvg(LDR_RIGHT);
  int bottom = readAvg(LDR_BOTTOM);
  int left   = readAvg(LDR_LEFT);

  // Compute errors
  int errPan  = left - right;   // + means sun on LEFT
  int errTilt = top  - bottom;  // + means sun on TOP

  // PAN movement
  if (abs(errPan) > DEAD_BAND) {
    if (errPan > 0)
      panAngle += STEP;
    else
      panAngle -= STEP;
  }

  // TILT movement
  if (abs(errTilt) > DEAD_BAND) {
    if (errTilt > 0)
      tiltAngle += STEP;
    else
      tiltAngle -= STEP;
  }

  // Keep within limits
  panAngle  = clampi(panAngle,  PAN_MIN,  PAN_MAX);
  tiltAngle = clampi(tiltAngle, TILT_MIN, TILT_MAX);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  // Debug monitor
  Serial.print("T:"); Serial.print(top);
  Serial.print(" R:"); Serial.print(right);
  Serial.print(" B:"); Serial.print(bottom);
  Serial.print(" L:"); Serial.print(left);
  Serial.print(" | errP:"); Serial.print(errPan);
  Serial.print(" errT:"); Serial.print(errTilt);
  Serial.print(" | pan:"); Serial.print(panAngle);
  Serial.print(" tilt:"); Serial.println(tiltAngle);

  delay(LOOP_MS);
}
