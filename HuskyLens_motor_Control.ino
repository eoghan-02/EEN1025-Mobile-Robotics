#include <Wire.h>
#include "HUSKYLENS.h"

#define CW  LOW
#define ACW HIGH

//Pin Definitions 
int Red1 = 19;
int Blue1 = 42;
int BUZZER_PIN = 2;

int motor1PWM   = 37; // LHS motor
int motor1Phase = 38;
int motor2PWM   = 39; // RHS motor
int motor2Phase = 20;

HUSKYLENS huskylens;
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;

bool huskyOK = false;
uint32_t lastHuskyTryMs = 0;

//LED logic variables 
unsigned long previousMillis = 0;
unsigned long interval = 200;
bool RedledState = LOW;
bool BlueledState = HIGH;
int ledCount = 0;

// ===== TUNING =====
const int X_CENTER = 160;

// Increase this to reduce twitch near center
const int DEAD_BAND = 18;      // pixels

// Lower Kp reduces hunting
float Kp = 0.50f;                

// Base speed 
int baseSpeed = 100;

// Hard limit on steering strength
int maxSteer = 35;

// Smoothing (0..1). Higher = more smoothing but more lag.
const float ALPHA = 0.25f;

// Limit steer change per loop (damps oscillation)
const int STEER_SLEW = 4;         // max change per loop step

// ==================
float xFilt = X_CENTER;
int steerPrev = 0;

// ===== LOST-COAST (keep speed up) =====
bool lostMode = false;
bool everHadTarget = false;   // only allow searching after first lock-on

uint32_t lostSinceMs = 0;

int lastSeenX = X_CENTER;
int lastDir = 0;                // -1 left, +1 right (where target last was)
int lastLeftSpeed = 0;
int lastRightSpeed = 0;

const uint32_t LOST_GRACE_MS = 80;     // ignore 1-2 bad frames
const uint32_t LOST_HARD_MS  = 2500;   // after this, start sweeping harder

const int BIAS_START = 6;              // small initial steer bias
const int BIAS_MAX   = 35;             // max extra bias (keep <= maxSteer-ish)
const int BIAS_RAMP_PER_SEC = 10;      // how fast bias grows when lost


//SIREN SETUP 
// ===== PWM carrier =====
const uint32_t CARRIER_HZ = 20000;
const uint8_t  PWM_RES   = 10;

// ===== WAIL settings =====
const uint16_t DUTY_MIN = 75;
const uint16_t DUTY_MAX = 900;
const uint16_t STEP     = 8;
const uint32_t STEP_MS  = 2;

// ===== YELP settings =====
const uint32_t YELP_INTERVAL_MS = 4000;  // how often yelp occurs
const uint32_t YELP_DURATION_MS = 800;   // how long yelp lasts
const uint32_t YELP_PULSE_MS    = 30;    // yelp aggressiveness 


bool ensureHuskyReady() {
  // already good
  if (huskyOK) return true;

  // retry at most every 250ms (avoid hammering I2C)
  uint32_t now = millis();
  if (now - lastHuskyTryMs < 250) return false;
  lastHuskyTryMs = now;

  // (re)start I2C and try to connect
  Wire.end();
  delay(5);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);

  if (!huskylens.begin(Wire)) {
    huskyOK = false;
    return false;
  }

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
  huskyOK = true;
  return true;
}

//Siren Functions 
void sirenUpdate(bool enable) { 
  static bool init = false;

  // Wail state
  static uint16_t duty = DUTY_MIN;
  static int dir = +1;
  static uint32_t lastStep = 0;

  // Yelp state
  static bool yelpActive = false;
  static bool yelpOn = false;
  static uint32_t lastYelp = 0;
  static uint32_t lastPulse = 0;

  if (!enable) {
    if (init) ledcWrite(BUZZER_PIN, 0);
    return;
  }

  if (!init) {
    ledcAttach(BUZZER_PIN, CARRIER_HZ, PWM_RES);
    ledcWrite(BUZZER_PIN, 0);
    lastStep = millis();
    lastYelp = millis();
    init = true;
  }

  uint32_t now = millis();

  // ---------- Yelp timing ----------
  if (!yelpActive && (now - lastYelp >= YELP_INTERVAL_MS)) {
    yelpActive = true;
    lastYelp = now;
    lastPulse = now;
    yelpOn = true;
  }

  if (yelpActive && (now - lastYelp >= YELP_DURATION_MS)) {
    yelpActive = false;
    ledcWrite(BUZZER_PIN, duty);  // resume wail level
  }

  // ---------- Yelp behaviour ----------
  if (yelpActive) {
    if (now - lastPulse >= YELP_PULSE_MS) {
      lastPulse = now;
      yelpOn = !yelpOn;
      ledcWrite(BUZZER_PIN, yelpOn ? DUTY_MAX : 0);
    }
    return; // yelp overrides wail
  }

  // ---------- Wail behaviour ----------
  if (now - lastStep >= STEP_MS) {
    lastStep = now;

    int next = (int)duty + dir * STEP;
    if (next >= DUTY_MAX) { next = DUTY_MAX; dir = -1; }
    if (next <= DUTY_MIN) { next = DUTY_MIN; dir = +1; }
    duty = (uint16_t)next;

    ledcWrite(BUZZER_PIN, duty);
  }
}

void flashing(bool enable) {

  if(!enable){
    digitalWrite(Red1, LOW);
    digitalWrite(Blue1, LOW);

    ledCount = 0;
    interval = 200;
    
    return;
  } 

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;      // save the last time
    RedledState = !RedledState;
    BlueledState = !BlueledState;                 // toggle state
    digitalWrite(Red1, RedledState);
    digitalWrite(Blue1, BlueledState);

    ledCount++;

    if (ledCount >= 5) {        
      interval = 50;            // quick bursts
    }
    if (ledCount >= 15) {       
      interval = 200;           // slower flash
      ledCount = 0;             // restart cycle
  }

  }
}

void stopMotors() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}


bool getFirstBlockX(int &x) {
  if (!huskylens.request()) {
    huskyOK = false;          // <-- IMPORTANT: force re-init attempts
    return false;
  }
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
  pinMode(Red1, OUTPUT);
  pinMode(Blue1, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(600);                 // important for battery boot
  ensureHuskyReady();         // try once
}

void loop() {

  if (!ensureHuskyReady()) {
  // Husky not ready yet -> stay safe
  stopMotors();
  sirenUpdate(false);
  flashing(false);
  delay(30);
  return;
  }

  int xRaw;
  bool hasTarget = getFirstBlockX(xRaw);

  sirenUpdate(hasTarget);   // siren when target found
  flashing(hasTarget); 

  uint32_t now = millis();

  if (!hasTarget) {
    // Start lost timer once
    if (!everHadTarget) {
    stopMotors();
    sirenUpdate(false);
    flashing(false);
    Serial.println("No target yet (idle)");
    delay(30);
    return;
  }

    if (!lostMode) {
      lostMode = true;
      lostSinceMs = now;

      // If lastDir is unknown (target was centered), pick a default
      if (lastDir == 0) lastDir = +1;
    }

    // Brief grace: just keep EXACT last speeds (no changes)
    if (now - lostSinceMs < LOST_GRACE_MS) {
      digitalWrite(motor1Phase, ACW);
      digitalWrite(motor2Phase, CW);
      analogWrite(motor1PWM, lastLeftSpeed);
      analogWrite(motor2PWM, lastRightSpeed);
      delay(30);
      return;
    }

    // After grace: keep speed high, add a growing bias to arc toward last seen side
    float lostSec = (now - lostSinceMs) / 1000.0f;
    int biasMag = constrain(BIAS_START + (int)(lostSec * BIAS_RAMP_PER_SEC), BIAS_START, BIAS_MAX);
    int bias = lastDir * biasMag;

    // Optional: after a long time lost, sweep harder by flipping bias periodically
    if (now - lostSinceMs > LOST_HARD_MS) {
      if (((now - lostSinceMs) / 900) % 2 == 1) bias = -bias;
    }

    int leftSpeed  = constrain(lastLeftSpeed  - bias, 0, 255);
    int rightSpeed = constrain(lastRightSpeed + bias, 0, 255);

    digitalWrite(motor1Phase, ACW);
    digitalWrite(motor2Phase, CW);
    analogWrite(motor1PWM, leftSpeed);
    analogWrite(motor2PWM, rightSpeed);

    Serial.print("LOST coast L=");
    Serial.print(lastLeftSpeed);
    Serial.print(" R=");
    Serial.print(lastRightSpeed);
    Serial.print(" bias=");
    Serial.println(bias);

    delay(30);
    return;
  }

  everHadTarget = true;
  // 1) Smooth xCenter
  xFilt = (1.0f - ALPHA) * xFilt + ALPHA * (float)xRaw;

  // 2) Error + deadband
  float err = xFilt - X_CENTER;
  if (abs((int)err) < DEAD_BAND) err = 0;

  // 3) Steering 
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

    // Save last good chase state
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;

  lastSeenX = xRaw;
  if (lastSeenX < X_CENTER) lastDir = -1;
  else if (lastSeenX > X_CENTER) lastDir = +1;

  lostMode = false;   // reset lost state when we see target


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
