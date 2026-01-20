#define CW  LOW
#define ACW HIGH

// Motor pins
int motor1PWM   = 37; // LHS motor
int motor1Phase = 38;
int motor2PWM   = 39; // RHS motor
int motor2Phase = 20;

// Soft-start for base speed
int baseSpeedCurrent = 0;     // ramped base speed 
const int rampStep   = 2;     // PWM increment per   

// Line sensor pins (left → right)
int AnalogValue[5] = {0, 0, 0, 0, 0};
int AnalogPin[5]   = {4, 5, 6, 7, 15};  // TCRT5000 outputs

// ===== PID variables for line following =====
float Kp = 70.0;   
float Ki = 0.0;
float Kd = 12.0;

float error        = 0;
float previousError = 0;
float integral      = 0;

// Target base speed 
int baseSpeedTarget = 180;    // 0–255
bool lineLost = false;
int lastTurnDir = -1;

void setup() {
  Serial.begin(9600);

  pinMode(motor1PWM,   OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM,   OUTPUT);
  pinMode(motor2Phase, OUTPUT);
}

void loop() {
  // 1) Read sensors (and print for debugging)
  readSensorsAndPrint();

  // 2) Follow line with PID
  lineFollowStep();
}

/* -------------------- SENSOR + PID FUNCTIONS -------------------- */

void readSensorsAndPrint() {
  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    Serial.print(AnalogValue[i]);
    Serial.print("\t");
  }
  Serial.println("");
}

// Convert 5 sensor readings into a line position error
float getLineError() {
  // weights: left-most = -2, right-most = +2
  const int weights[5] = {-2, -1, 0, 1, 2};

  long numerator   = 0;
  long denominator = 0;

  for (int i = 0; i < 5; i++) {
    int value = AnalogValue[i];

    // If line is dark, the sensor reading is LOWER 
    // Must Invert so that "more line" = bigger value:
    value = 4095 - value;    // ESP32 12-bit ADC (0–4095) 

    numerator   += (long)value * weights[i];
    denominator += value;
  }

  if (denominator<4000) {
    // No line detected 
    lineLost = true;
    return 0;
  }
  lineLost = false;
  float pos = (float)numerator / (float)denominator;  // range approx -2 … +2

  if(pos>0){
    lastTurnDir = 1; //line on the rhs
  }
  else if (pos<0){
    lastTurnDir =-1; //line on lhs 
  }
  return pos;
}

float computePID(float lineError) {
  integral += lineError;
  // prevent integral windup
  integral = constrain(integral, -50, 50);

  float derivative = lineError - previousError;
  previousError = lineError;

  // Full PID – you can leave Ki = 0 and it becomes PD
  float output = Kp * lineError + Ki * integral + Kd * derivative;
  return output;
}

// One step of line following: call this every loop()
void lineFollowStep() {

  // --- 1) Update line error ---
  float lineError   = getLineError();

  if(lineLost){
    searchForLine();
    return;
  }
  float correction  = computePID(lineError);

  // --- 2) Soft-start base speed (re-using your ramp idea) ---
  if (baseSpeedCurrent < baseSpeedTarget) {
    baseSpeedCurrent += rampStep;
    if (baseSpeedCurrent > baseSpeedTarget) baseSpeedCurrent = baseSpeedTarget;
  } else if (baseSpeedCurrent > baseSpeedTarget) {
    baseSpeedCurrent -= rampStep;
    if (baseSpeedCurrent < baseSpeedTarget) baseSpeedCurrent = baseSpeedTarget;
  }

  // --- 3) Differential speeds from PID correction ---
  int leftSpeed  = baseSpeedCurrent + (int)correction;
  int rightSpeed = baseSpeedCurrent - (int)correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // --- 4) Drive motors forward with those speeds ---
  digitalWrite(motor1Phase, ACW); // LHS forward
  digitalWrite(motor2Phase, CW);  // RHS forward

  analogWrite(motor1PWM, leftSpeed);
  analogWrite(motor2PWM, rightSpeed);

  // Debug (optional)
  // Serial.print("Err: "); Serial.print(lineError);
  // Serial.print("  L: "); Serial.print(leftSpeed);
  // Serial.print("  R: "); Serial.println(rightSpeed);
}

/* -------------------- OTHER MOTION FUNCTIONS (OPTIONAL) -------------------- */

void Brake() {
  baseSpeedCurrent = 0;
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}

void spinRight(int speed){

  digitalWrite(motor1Phase, ACW ); // LHS forward
  digitalWrite(motor2Phase, ACW); // RHS BACK
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  Serial.println("Im Dizzy (RHS)"); // Display motor direction

}

void spinLeft(int speed){

  digitalWrite(motor1Phase, CW); //LHS back
  digitalWrite(motor2Phase, CW); //RHS forward
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  Serial.println("Im Dizzy (LHS)"); // Display motor direction
}

void searchForLine(){ 
  Serial.print("Searching");
  int searchSpeed = 60; 
  if (lastTurnDir>0){
   // analogWrite(motor1PWM, 150); //Keep Turning Right 
   // analogWrite(motor2PWM, 30);
   spinRight(searchSpeed);
    }
  else { 
   // analogWrite(motor1PWM, 30); //Keep Turning Left 
   // analogWrite(motor2PWM, 150);
   spinLeft(searchSpeed);
  }

}

