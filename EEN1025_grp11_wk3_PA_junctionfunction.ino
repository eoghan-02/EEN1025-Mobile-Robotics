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


//=======================CALIBRATION VARIABLES==============================

//Light levels 
const int nodeLightLevel = 2500;
const int lostLightLevel = 800;

//Speed
int baseSpeedTarget = 180;

//PID variables 
float Kp = 90;   
float Ki = 0.0;
float Kd = 12.0;

//============================================================================

float error        = 0;
float previousError = 0;
float integral      = 0;

bool lineLost = false;
int lastTurnDir = -1;
int nodeCount = 0;

//================ STATE MACHINE FOR ROUTE =================
enum CaseState {CASE_0, CASE_1, CASE_2, CASE_3, CASE_4};
CaseState currentCase = CASE_0;
CaseState lastCase    = CASE_0;

// Predefined route (edit to test different paths)
CaseState route[] = {CASE_1, CASE_4, CASE_3, CASE_0, CASE_2};
int routeIndex = 0;

int nodeCounter = 0;
bool actionInProgress = false;
bool startedRoute = false;
//===========================================================

unsigned long lastNodeTime = 0;
const int NODE_DEBOUNCE_MS = 350;   // ms
const int NODE_COUNT_TH = 4;        // >=4 sensors see line => node marker

void advanceCase(CaseState nextCase, bool enteringFromStart = false);

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

  // Detect the first node for starting CASE 0
  if (!startedRoute && nodeDetected()) {  // first node only once
    advanceCase(CASE_0, true);              // enter CASE 0 without flipping
    nodeCounter = 0;   // reset count after entering CASE 0
    Serial.println("First node detected, entering CASE 0 | nodeCounter reset to 0");
    startedRoute = true;
  }

  // Normal node handling after first CASE
  if (startedRoute && nodeDetected()) {
    nodeCounter++;
    Serial.print("Node hit: ");
    Serial.println(nodeCounter);
    handleNodeLogic();
  }

  if (!actionInProgress) {
    lineFollowStep();
  }
}

//SENSOR + PID FUNCTIONS 

void readSensorsAndPrint() {
  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
  }
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

  lineLost = lineLostFun();

  if (lineLost) return previousError;

  if (denominator == 0) return 0;
  
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

}

//Motor Movement Functions 

void Brake(int time) {
  baseSpeedCurrent = 0;
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  delay(time);
}

void spinRight(int speed){

  digitalWrite(motor1Phase, ACW ); // LHS forward
  digitalWrite(motor2Phase, ACW); // RHS BACK
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  // Serial.println("Im Dizzy (RHS)"); // Display motor direction

}

void spinLeft(int speed){

  digitalWrite(motor1Phase, CW); //LHS back
  digitalWrite(motor2Phase, CW); //RHS forward
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  // Serial.println("Im Dizzy (LHS)"); // Display motor direction
}

void searchForLine(){ 
  Serial.print("Searching");
  int searchSpeed = 120; 
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

int countSensorsOnLine() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    int v = 4095 - AnalogValue[i];     // invert
    if (v > nodeLightLevel) count++;
  }
  return count;
}

bool lineLostFun(){
  int cnt = 0;

  for (int i=0; i<5; i++){
    int inv = 4095- AnalogValue[i]; 
    if (inv < lostLightLevel) cnt++;

    if (cnt == 5){
      return true;
    } 
  }
  return false;
}

bool nodeDetected() {
  if (millis() - lastNodeTime < NODE_DEBOUNCE_MS) return false;

  int c = countSensorsOnLine();
  if (c >= 4) {
    lastNodeTime = millis();
    return true;
  }
  return false;
}

//==================== ROUTE CONTROL LOGIC ====================

void handleNodeLogic() {
  if (routeIndex >= sizeof(route)/sizeof(route[0])) return;

  CaseState nextCase = route[routeIndex];

  Serial.print("Current CASE: "); Serial.print(currentCase);
  Serial.print(" | Next CASE: "); Serial.print(nextCase);
  Serial.print(" | nodeCounter: "); Serial.println(nodeCounter);

  bool didAdvanceCase = false;
  switch (currentCase) {

    case CASE_0:

    // Example: travel logic
    if (nextCase == CASE_1) {
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnLeft90();      // first node turn
          Serial.println("Turning Left");
        }
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(nextCase); // after 2nd node, switch CASE
          didAdvanceCase = true;
        }
    }
    else if (nextCase == CASE_2) {
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(nextCase);
          didAdvanceCase = true;
        }
    }
    else if (nextCase == CASE_3) {
        if (nodeCounter == 3) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(nextCase);
          didAdvanceCase = true;
        }
    }
    else if (nextCase == CASE_4) {
        flip180();
        if (nodeCounter == 1) {
        continueForwardShort(); // clear node before CASE switch
        advanceCase(nextCase);
        didAdvanceCase = true;
        }
    }

    break;

    case CASE_1:

      // -------- next.case = 0 --------
      if (nextCase == CASE_0) {

        // node.count = 1 → TURN RIGHT
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnRight90();
          Serial.println("Turning Right");
        }

        // node.count = 2 → CONTINUE, FLIP 180
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before flip
          flip180();
        }
        // node.count = 3 → go to CASE 0
        if (nodeCounter == 3) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_0);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 2 --------
      else if (nextCase == CASE_2) {

        // node.count = 1 → TURN LEFT
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnLeft90();
          Serial.println("Turning Left");
       }
        // node.count = 2 → go to CASE 2
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_2);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 3 --------
      else if (nextCase == CASE_3) {

        // node.count = 1 → TURN RIGHT
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnRight90();
          Serial.println("Turning Right");
        }
        // node.count = 2 → go to CASE 3
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_3);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 4 --------
      else if (nextCase == CASE_4) {

        // node.count = 1 → TURN LEFT
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnLeft90();
          Serial.println("Turning Left");
        }
        // node.count = 2 → go to CASE 4
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_4);
          didAdvanceCase = true;
        }
      }

    break;

    case CASE_2:

      // -------- next.case = 0 --------
      if (nextCase == CASE_0) {

        // node.count = 2 → CONTINUE and FLIP
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before flip
          flip180();
        }

        // node.count = 3 → FLIP 180 then go to CASE 0
        if (nodeCounter == 3) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_0);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 1 --------
      else if (nextCase == CASE_1) {

        // node.count = 1 → TURN RIGHT
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnRight90();
          Serial.println("Turning Right");
        }

        // node.count = 2 → go to CASE 1
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_1);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 3 --------
      else if (nextCase == CASE_3) {

        // node.count = 1 → go to CASE 3 (straight transition)
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_3);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 4 --------
      else if (nextCase == CASE_4) {

        // node.count = 3 → go to CASE 4
        if (nodeCounter == 3) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_4);
          didAdvanceCase = true;
        }
      }

    break;

    case CASE_3:

  // -------- next.case = 0 --------
  if (nextCase == CASE_0) {

    // node.count = 3 → go to CASE 0
    if (nodeCounter == 3) {
      continueForwardShort(); // clear node before CASE switch
      advanceCase(CASE_0);
      didAdvanceCase = true;
    }
  }

  // -------- next.case = 1 --------
  else if (nextCase == CASE_1) {

    // node.count = 1 → TURN LEFT
    if (nodeCounter == 1) {
      continueForwardShort(); // clear node first
      turnLeft90();
      Serial.println("Turning Left");
    }
    // node.count = 2 → go to CASE 1
    if (nodeCounter == 2) {
      continueForwardShort(); // clear node before CASE switch
      advanceCase(CASE_1);
      didAdvanceCase = true;
    }
  }

  // -------- next.case = 2 --------
  else if (nextCase == CASE_2) {

    // node.count = 1 → go to CASE 2
    if (nodeCounter == 1) {
      continueForwardShort(); // clear node before CASE switch
      advanceCase(CASE_2);
      didAdvanceCase = true;
    }
  }

  // -------- next.case = 4 --------
  else if (nextCase == CASE_4) {

    // node.count = 2 → go to CASE 4
    if (nodeCounter == 2) {
      continueForwardShort(); // clear node before CASE switch
      advanceCase(CASE_4);
      didAdvanceCase = true;
    }
  }

break;

    case CASE_4:

      // -------- next.case = 0 --------
      if (nextCase == CASE_0) {

        // node.count = 1 → go to CASE 0
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_0);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 1 --------
      else if (nextCase == CASE_1) {

        // node.count = 1 → TURN RIGHT
        if (nodeCounter == 1) {
          continueForwardShort(); // clear node first
          turnRight90();
          Serial.println("Turning Right");
        }
         // node.count = 2 → go to CASE 1
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_1);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 2 --------
      else if (nextCase == CASE_2) {

        // node.count = 3 → go to CASE 2
        if (nodeCounter == 3) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_2);
          didAdvanceCase = true;
        }
      }

      // -------- next.case = 3 --------
      else if (nextCase == CASE_3) {

        // node.count = 2 → go to CASE 3
        if (nodeCounter == 2) {
          continueForwardShort(); // clear node before CASE switch
          advanceCase(CASE_3);
          didAdvanceCase = true;
        }
      }

    break;
  }

// Step 3: Small forward move after CASE transition (physically leave the junction)
if (didAdvanceCase) {
  routeIndex++;           // <-- MOVE routeIndex increment here
  continueForwardShort();
}
}

void advanceCase(CaseState nextCase, bool enteringFromStart) {

  // Save previous CASE
  lastCase = currentCase;
  CaseState previousCase = currentCase;  

  // Update current CASE
  currentCase = nextCase;

  // Determine next CASE in the route (upcoming)
  CaseState upcomingCase = (routeIndex + 1 < sizeof(route)/sizeof(route[0])) ? route[routeIndex + 1] : currentCase;

  // ===== REORIENTATION PHASE (happens BEFORE node counting) =====
  // Skip flips if entering from start
  if (!enteringFromStart) {
    switch (currentCase) {

      case CASE_1:
        if (upcomingCase == CASE_0) {
          if (lastCase == CASE_0 || lastCase == CASE_2) flip180();
        }
        else if (upcomingCase == CASE_2) {
          if (lastCase == CASE_0 || lastCase == CASE_2) flip180();
        }
        else if (upcomingCase == CASE_3) {
          if (lastCase == CASE_3 || lastCase == CASE_4) flip180();
        }
        else if (upcomingCase == CASE_4) {
          if (lastCase == CASE_3 || lastCase == CASE_4) flip180();
        }
        break;

      case CASE_2:
        if (upcomingCase == CASE_0) {
          if (lastCase == CASE_0 || lastCase == CASE_1 || lastCase == CASE_4) flip180();
        }
        else if (upcomingCase == CASE_1) {
          if (lastCase == CASE_0 || lastCase == CASE_1 || lastCase == CASE_4) flip180();
        }
        else if (upcomingCase == CASE_3) {
          if (lastCase == CASE_3) flip180();
        }
        else if (upcomingCase == CASE_4) {
          if (lastCase == CASE_3) flip180();
        }
        break;

      case CASE_3:
        if (upcomingCase == CASE_0) {
          if (lastCase == CASE_1 || lastCase == CASE_4) flip180();
        }
        else if (upcomingCase == CASE_1) {
          if (lastCase == CASE_1 || lastCase == CASE_4) flip180();
        }
        else if (upcomingCase == CASE_2) {
          if (lastCase == CASE_0 || lastCase == CASE_2) flip180();
        }
        else if (upcomingCase == CASE_4) {
          if (lastCase == CASE_1 || lastCase == CASE_4) flip180();
        }
        break;

      case CASE_4:
        if (upcomingCase == CASE_0) {
          if (lastCase == CASE_0) flip180();
        }
        else if (upcomingCase == CASE_1) {
          if (lastCase == CASE_1 || lastCase == CASE_2 || lastCase == CASE_3) flip180();
        }
        else if (upcomingCase == CASE_2) {
          if (lastCase == CASE_0) flip180();
        }
        else if (upcomingCase == CASE_3) {
          if (lastCase == CASE_1 || lastCase == CASE_2 || lastCase == CASE_3) flip180();
        }
        break;

      default:
        break;
    }
  }

  // Reset node counter for travel phase
  nodeCounter = 0;

  Serial.print("Switching to CASE ");
  Serial.println(currentCase);
}

//==================== MOVEMENT ACTIONS ====================

void turnLeft90() {
  actionInProgress = true;
  spinLeft(120);                // slower turn
  delay(500);                  // adjust delay for slower spin
  Brake(50);
  actionInProgress = false;
}

void turnRight90() {
  actionInProgress = true;
  spinRight(120);               // slower turn
  delay(500);                  // adjust delay for slower spin
  Brake(50);
  actionInProgress = false;
}

void flip180() {
  Serial.println("Flipping Around");  // new message
  actionInProgress = true;
  spinRight(170);
  delay(900);
  Brake(50);
  actionInProgress = false;
}

void continueForwardShort() {
  actionInProgress = true;
  digitalWrite(motor1Phase, ACW);
  digitalWrite(motor2Phase, CW);
  analogWrite(motor1PWM, 140);
  analogWrite(motor2PWM, 140);
  delay(250);
  Brake(50);
  actionInProgress = false;
}

//============================================================