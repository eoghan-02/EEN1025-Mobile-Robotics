#include <WiFi.h>

// ======================= WIFI + SERVER CONFIG =======================
static char ssid[]     = "Pele16Pro";
static char password[] = "cursethis";

static WiFiClient client;
static char serverHost[] = "3.250.38.184";
static int  serverPort    = 8000;

static const char ROBOT_ID[] = "pllk3098";

#define READ_TIMEOUT_MS 2000
// ===================================================================

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
enum CaseState {CASE_0, CASE_1, CASE_2, CASE_3, CASE_4, CASE_5};
CaseState currentCase = CASE_0;
CaseState lastCase    = CASE_0;

// Route received from server (dynamic)
CaseState *route = nullptr;
int routeLength = 0;
int routeIndex  = 0;

int nodeCounter = 0;
bool actionInProgress = false;
bool startedRoute = false;
bool routeReady = false;   // becomes true only after a valid route is fetched and assigned
//===========================================================

unsigned long lastNodeTime = 0;
const int NODE_DEBOUNCE_MS = 350;   // ms
const int NODE_COUNT_TH = 4;        // >=4 sensors see line => node marker

void advanceCase(CaseState nextCase, bool enteringFromStart = false);

void handleNodeLogic();

// Movement action forward declarations (used before their definitions)
void Brake(int time);
void spinRight(int speed);
void spinLeft(int speed);
void turnLeft90();
void turnRight90();
void flip180();
void continueForwardShort();

// ======================= WIFI + SERVER HELPERS =======================
void connectToWiFi() {
  Serial.print("Connecting to network: ");
  Serial.print(ssid);
  Serial.flush();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    Serial.flush();
    delay(300);
  }
  Serial.println("\nConnected to WiFi");

  Serial.print("Obtaining IP address");
  Serial.flush();
  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");
    Serial.flush();
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

bool connectServer() {
  if (client.connected()) return true;
  if (!client.connect(serverHost, serverPort)) {
    Serial.println("error connecting to server");
    return false;
  }
  return true;
}

String readLineWithTimeout(WiFiClient &c, unsigned long timeoutMs) {
  String line = "";
  unsigned long start = millis();
  while (true) {
    while (c.available()) {
      char ch = c.read();
      line += ch;
      if (ch == '\n') {
        line.trim();
        return line;
      }
    }
    if (millis() - start > timeoutMs) {
      line.trim();
      return line;
    }
    delay(1);
  }
}

bool readHttpResponse(int &statusCode, String &body) {
  statusCode = 0;
  body = "";
  if (!client.connected()) return false;

  String statusLine = readLineWithTimeout(client, READ_TIMEOUT_MS);
  if (statusLine.length() == 0) return false;

  if (statusLine.startsWith("HTTP/")) {
    int firstSpace = statusLine.indexOf(' ');
    if (firstSpace >= 0) {
      int secondSpace = statusLine.indexOf(' ', firstSpace + 1);
      if (secondSpace > firstSpace) {
        String code = statusLine.substring(firstSpace + 1, secondSpace);
        statusCode = code.toInt();
      }
    }
  }

  int contentLength = -1;
  while (true) {
    String header = readLineWithTimeout(client, READ_TIMEOUT_MS);
    if (header.length() == 0) break;
    String h = header;
    h.toLowerCase();
    if (h.startsWith("content-length:")) {
      int colon = header.indexOf(':');
      if (colon >= 0) {
        String val = header.substring(colon + 1);
        val.trim();
        contentLength = val.toInt();
      }
    }
  }

  if (contentLength >= 0) {
    char *buf = new char[contentLength + 1];
    int readSoFar = 0;
    unsigned long start = millis();
    while (readSoFar < contentLength && millis() - start < READ_TIMEOUT_MS) {
      while (client.available() && readSoFar < contentLength) {
        int toRead = client.readBytes(buf + readSoFar, contentLength - readSoFar);
        if (toRead <= 0) break;
        readSoFar += toRead;
      }
      delay(1);
    }
    buf[readSoFar] = 0;
    body = String(buf);
    delete[] buf;
  } else {
    unsigned long start = millis();
    while (millis() - start < READ_TIMEOUT_MS) {
      while (client.available()) {
        body += (char)client.read();
      }
      if (body.length() > 0) break;
      delay(1);
    }
    body.trim();
  }

  body.trim();
  return true;
}

void sendArrivalToServer(int position) {
  if (!connectServer()) return;

  String postBody = "position=" + String(position);

  client.println(String("POST /api/arrived/") + ROBOT_ID + " HTTP/1.1");
  client.println("Host: " + String(serverHost));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println();
  client.println(postBody);

  Serial.println("Reported arrival at position " + String(position) + " to server.");

  // Best-effort drain (prevents buffer buildup)
  int code;
  String body;
  readHttpResponse(code, body);
}

// Optional: adjust endpoint/body to whatever your server expects for “finished”.
void sendFinishedToServer(int finalPosition) {
  if (!connectServer()) return;

  String postBody = "position=" + String(finalPosition) + "&finished=1";

  client.println(String("POST /api/finished/") + ROBOT_ID + " HTTP/1.1");
  client.println("Host: " + String(serverHost));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println();
  client.println(postBody);

  Serial.println("Reported FINISHED at position " + String(finalPosition) + " to server.");

  int code;
  String body;
  readHttpResponse(code, body);
}

CaseState intToCaseState(int v) {
  switch (v) {
    case 0: return CASE_0;
    case 1: return CASE_1;
    case 2: return CASE_2;
    case 3: return CASE_3;
    case 4: return CASE_4;
    default: return CASE_5; // v==5 only, per your guarantee
  }
}

// Fetch route from server and populate `route`/`routeLength`.
// If the FIRST token is 0, we SKIP inserting it (CASE_0 is assumed at start).
bool fetchRouteFromServer() {
  if (!connectServer()) return false;

  client.println(String("GET /api/getRoute/") + ROBOT_ID + " HTTP/1.1");
  client.println("Host: " + String(serverHost));
  client.println("Connection: keep-alive");
  client.println();

  Serial.println("Asking the server what my next stops are...");

  String responseBody;
  int statusCode;
  if (!readHttpResponse(statusCode, responseBody)) {
    Serial.println("No proper reply from server.");
    return false;
  }

  Serial.println("Server says: " + responseBody);
  Serial.println("HTTP status code: " + String(statusCode));

  if (statusCode != 200) return false;

  if (responseBody.startsWith("Finished") || responseBody.startsWith("Already Finished")) {
    Serial.println("Server indicates route already finished.");
    routeLength = 0;
    return true;
  }

  // First pass: count tokens
  int tokenCount = 0;
  int startIdx = 0;
  while (startIdx < responseBody.length()) {
    int commaIdx = responseBody.indexOf(',', startIdx);
    String posStr;
    if (commaIdx == -1) {
      posStr = responseBody.substring(startIdx);
      startIdx = responseBody.length();
    } else {
      posStr = responseBody.substring(startIdx, commaIdx);
      startIdx = commaIdx + 1;
    }
    posStr.trim();
    if (posStr.length() == 0) continue;
    tokenCount++;
  }

  if (tokenCount <= 0) {
    routeLength = 0;
    return true;
  }

  // Second pass: parse and fill, skipping a leading 0
  CaseState *tmp = new CaseState[tokenCount];
  int tmpLen = 0;

  startIdx = 0;
  int tokenIdx = 0;
  while (startIdx < responseBody.length()) {
    int commaIdx = responseBody.indexOf(',', startIdx);
    String posStr;
    if (commaIdx == -1) {
      posStr = responseBody.substring(startIdx);
      startIdx = responseBody.length();
    } else {
      posStr = responseBody.substring(startIdx, commaIdx);
      startIdx = commaIdx + 1;
    }
    posStr.trim();
    if (posStr.length() == 0) continue;

    int position = posStr.toInt();

    if (tokenIdx == 0 && position == 0) { // skip only if first token is 0
      tokenIdx++;
      continue;
    }

    tmp[tmpLen++] = intToCaseState(position);
    tokenIdx++;
  }

  if (route != nullptr) {
    delete[] route;
    route = nullptr;
  }

  route = tmp;
  routeLength = tmpLen;
  routeIndex = 0;

  Serial.print("Route length (excluding leading 0 if present): ");
  Serial.println(routeLength);

  return true;
}

void stopAndHalt() {
  Serial.println("Route complete. Stopping motors and halting.");
  Brake(0);
  while (true) delay(1000);
}
// =======================================================================

void setup() {
  Serial.begin(9600);

  // Ensure motors are configured and OFF before doing anything else
  pinMode(motor1PWM,   OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM,   OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  Brake(0);

  delay(1000);
  connectToWiFi();

  bool ok = connectServer();
  while (!ok) ok = connectServer();
  Serial.println("Connected to server.");

  bool routeFetched = fetchRouteFromServer();
  while (!routeFetched) {
    Serial.println("Fetching route failed; retrying...");
    routeFetched = fetchRouteFromServer();
  }

  // Route is now received and assigned to CASEs
  routeReady = true;

  if (routeLength == 0) {
    Serial.println("No remaining route steps (server says finished or empty).");
    // If you want the robot to stop permanently when finished:
    // stopAndHalt();
  }
}

void loop() {
  // Do not start driving until route is received and assigned
  if (!routeReady) {
    Brake(0);
    return;
  }

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

  // Start driving (line following) after route is ready. This will carry the car to the first node.
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
  // Per your wiring: ACW should produce a RIGHT spin
  digitalWrite(motor1Phase, ACW);
  digitalWrite(motor2Phase, ACW);
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
}

void spinLeft(int speed){
  // Per your wiring: CW should produce a LEFT spin
  digitalWrite(motor1Phase, CW);
  digitalWrite(motor2Phase, CW);
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
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
  if (routeIndex >= routeLength) return;

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

        // node.count = 3 → go to CASE 0
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
  routeIndex++;           // advance to the next target in the route

  // Physically leave the junction after switching cases
  continueForwardShort();

  // If we have just completed the last CASE in the route, stop and report finish.
  if (routeIndex >= routeLength) {
    int finalPos = (int)currentCase;

    // Arrival already reported in advanceCase(); this is explicit completion.
    sendFinishedToServer(finalPos);

    stopAndHalt();
  }
}

}

void advanceCase(CaseState nextCase, bool enteringFromStart) {

  // Save previous CASE
  lastCase = currentCase;
  CaseState previousCase = currentCase;  

  // Update current CASE
  currentCase = nextCase;

  // Determine next CASE in the route (upcoming)
  CaseState upcomingCase = (routeIndex + 1 < routeLength) ? route[routeIndex + 1] : currentCase;

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
  // Notify the server when we arrive at the start of each CASE_X (including CASE_0 at start)
  sendArrivalToServer((int)currentCase);
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