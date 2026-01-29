#include <Wire.h>
#include "HUSKYLENS.h"

#define CW  LOW
#define ACW HIGH

int motor1PWM   = 37; // LHS motor
int motor1Phase = 38;
int motor2PWM   = 39; // RHS motor
int motor2Phase = 20;

HUSKYLENS huskylens;
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;

// ===== TUNING =====
const int X_CENTER = 160;

// Increase this to reduce twitch near center
const int DEAD_BAND = 12;        // pixels

// Lower Kp reduces hunting
float Kp = 0.55f;                // start here (you likely had ~1.0+)

// Base speed (keep modest while tuning)
int baseSpeed = 60;

// Hard limit on steering strength
int maxSteer = 35;

// Smoothing (0..1). Higher = more smoothing but more lag.
// Try 0.15–0.30
const float ALPHA = 0.20f;

// Limit steer change per loop (damps oscillation)
const int STEER_SLEW = 6;         // max change per loop step

// ==================
float xFilt = X_CENTER;
int steerPrev = 0;

void stopMotors() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}

bool getFirstBlockX(int &x) {
  if (!huskylens.request()) return false;
  if (huskylens.count() == 0) return false;

  while (huskylens.available()) {
    HUSKYLENSResult r = huskylens.read();
    if (r.command == COMMAND_RETURN_BLOCK) {
      x = r.xCenter;
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens NOT found!");
    while (1);
  }

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
  Serial.println("Ready. Press LEARN on target.");
}

void loop() {
  int xRaw;
  if (!getFirstBlockX(xRaw)) {
    stopMotors();
    Serial.println("No target");
    delay(30);
    return;
  }

  // 1) Smooth xCenter
  xFilt = (1.0f - ALPHA) * xFilt + ALPHA * (float)xRaw;

  // 2) Error + deadband
  float err = xFilt - X_CENTER;
  if (abs((int)err) < DEAD_BAND) err = 0;

  // 3) Steering (note the minus sign you needed earlier)
  int steerCmd = (int)(-Kp * err);
  steerCmd = constrain(steerCmd, -maxSteer, maxSteer);

  // 4) Slew-rate limit (prevents rapid flip-flop)
  int delta = steerCmd - steerPrev;
  if (delta > STEER_SLEW) steerCmd = steerPrev + STEER_SLEW;
  if (delta < -STEER_SLEW) steerCmd = steerPrev - STEER_SLEW;
  steerPrev = steerCmd;

  // 5) Differential speeds
  int leftSpeed  = baseSpeed - steerCmd;
  int rightSpeed = baseSpeed + steerCmd;
  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(motor1Phase, ACW);
  digitalWrite(motor2Phase, CW);

  analogWrite(motor1PWM, leftSpeed);
  analogWrite(motor2PWM, rightSpeed);

  Serial.print("xRaw=");
  Serial.print(xRaw);
  Serial.print(" xFilt=");
  Serial.print((int)xFilt);
  Serial.print(" steer=");
  Serial.print(steerCmd);
  Serial.print(" L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);

  delay(30);
}
