#include <WiFi.h>
#include <queue>

// ======================= PERFORMANCE / DEBUG =======================
#ifndef DEBUG_LOG
#define DEBUG_LOG 0
#endif

#if DEBUG_LOG
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINT(x)   do {} while (0)
  #define DBG_PRINTLN(x) do {} while (0)
#endif
// ==================================================================

// ======================= WIFI + SERVER CONFIG =======================
static char ssid[]     = "iPhone";
static char password[] = "haider1432";

static WiFiClient client;
static char serverHost[] = "3.250.38.184";
static int  serverPort    = 8000;

static const char ROBOT_ID[] = "pllk3098";

#define READ_TIMEOUT_MS 2000
// ===================================================================

#define CW  LOW
#define ACW HIGH
enum USState {US_IDLE, US_TRIGGER, US_WAIT_ECHO_HIGH, US_WAIT_ECHO_LOW};
USState usState = US_IDLE;

unsigned long usTimer = 0;
unsigned long echoStart = 0;
float latestDistance = 999;
bool newReading = false;
unsigned long latestDistanceMs = 0; // millis() when latestDistance was updated
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
const int nodeLightLevel = 2700;
const int lostLightLevel = 800;

//Speed
int baseSpeedTarget = 180;

//PID variables 
float Kp = 120;   
float Ki = 0.0;
float Kd = 12.0;

//============================================================================

float error        = 0;
float previousError = 0;
float integral      = 0;

bool lineLost = false;
int lastTurnDir = -1;
int nodeCount = 0;

// Ultrasound sensor pins
const int trigPin = 16;
const int echoPin = 17;

// Sound speed in cm/us
#define SOUND_SPEED 0.034

//=================DETOUR HANDLING=====================

enum GraphNode {
  GN_NODE_0,
  GN_NODE_1,
  GN_NODE_2,
  GN_NODE_3,
  GN_NODE_4,
  GN_JUNC_A,   // between 0/1 and 2
  GN_JUNC_B,   // between 3/4 and 1
  GN_COUNT
};

const int MAX_GRAPH_NODES = GN_COUNT;

// CASE connections (0-indexed, 5 nodes)
int adjacencyMatrix[MAX_GRAPH_NODES][MAX_GRAPH_NODES] = {
    // 0  1  2  3  4 A B
    {0, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1, 1},
    {0, 0, 0, 1, 0, 1, 0},
    {0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 0, 0, 0, 0},
    {0, 1, 0, 1, 1, 0, 0}
};

// false = edge free, true = edge blocked
bool obstacleMatrix[MAX_GRAPH_NODES][MAX_GRAPH_NODES] = {
  {false, false, false, false, false, false, false},
  {false, false, false, false, false, false, false},
  {false, false, false, false, false, false, false},
  {false, false, false, false, false, false, false},
  {false, false, false, false, false, false, false},
  {false, false, false, false, false, false, false},
  {false, false, false, false, false, false, false}
};

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
bool parking = false;
int finalPos;
unsigned long parkingStart;
unsigned long elapsed;
unsigned long lastObstacleCheck = 0;
const unsigned long OBSTACLE_CHECK_INTERVAL = 80; // ms
bool ignoreNextNode = false;
int obstacleConfidence = 0;


// Maximum number of nodes
const int MAX_NODES = 5;

// Detour system
bool detourActive = false;
CaseState detourPath[MAX_NODES];
int detourLength = 0;
int detourIndex = 0;

// Map CaseState <-> GraphNode and helpers
GraphNode caseToGraph(CaseState c) {
  switch (c) {
    case CASE_0: return GN_NODE_0;
    case CASE_1: return GN_NODE_1;
    case CASE_2: return GN_NODE_2;
    case CASE_3: return GN_NODE_3;
    case CASE_4: return GN_NODE_4;
    default: return GN_NODE_0;
  }
}

CaseState graphToCase(int g) {
  switch (g) {
    case GN_NODE_0: return CASE_0;
    case GN_NODE_1: return CASE_1;
    case GN_NODE_2: return CASE_2;
    case GN_NODE_3: return CASE_3;
    case GN_NODE_4: return CASE_4;
    default: return CASE_0; // for junctions - should never be used directly
  }
}

bool isJunctionNode(int g) {
  return (g == GN_JUNC_A || g == GN_JUNC_B);
}

// Find a junction graph node that connects both a and b (returns -1 if none)
int findSharedJunction(int a, int b) {
  for (int j = 0; j < MAX_GRAPH_NODES; j++) {
    if (!isJunctionNode(j)) continue;
    if (adjacencyMatrix[j][a] && adjacencyMatrix[j][b]) return j;
  }
  return -1;
}
void ultrasonicTask() {

    switch (usState) {

        case US_IDLE:
            if (micros() - usTimer > 60000) { // every 60ms
                digitalWrite(trigPin, LOW);
                usTimer = micros();
                usState = US_TRIGGER;
            }
        break;

        case US_TRIGGER:
            if (micros() - usTimer >= 5) {
                digitalWrite(trigPin, HIGH);
                delayMicroseconds(10);
                digitalWrite(trigPin, LOW);
                usState = US_WAIT_ECHO_HIGH;
                usTimer = micros();
            }
        break;

        case US_WAIT_ECHO_HIGH:
            if (digitalRead(echoPin) == HIGH) {
                echoStart = micros();
                usState = US_WAIT_ECHO_LOW;
            }
            else if (micros() - usTimer > 20000) {
                usState = US_IDLE;
            }
        break;

        case US_WAIT_ECHO_LOW:
            if (digitalRead(echoPin) == LOW) {

                unsigned long duration = micros() - echoStart;
                float d = duration * 0.0343 / 2.0;

                // Accept close-in readings too; filtering happens at decision points
                if (d >= 2.0 && d < 200.0)
                    latestDistance = d;
                else
                    latestDistance = 999;

                latestDistanceMs = millis();
                newReading = true;
                usState = US_IDLE;
                usTimer = micros();
            }
            else if (micros() - echoStart > 20000) {
                usState = US_IDLE;
            }
        break;
    }
}


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
void turnRight20();
void turnLeft20();
void flip180();
void continueForwardShort();

// ======================= WIFI + SERVER HELPERS =======================
bool connectToWiFi(unsigned long timeoutMs = 20000) {
  Serial.print("Connecting to network: ");
  Serial.print(ssid);
  Serial.flush();

  WiFi.begin(ssid, password);
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) {
      Serial.println("\nWiFi connect timeout");
      return false;
    }
    Serial.print(".");
    Serial.flush();
    delay(300);
    yield();
  }

  Serial.println("\nConnected to WiFi");

  Serial.print("Obtaining IP address");
  Serial.flush();
  start = millis();
  while (WiFi.localIP() == INADDR_NONE) {
    if (millis() - start > timeoutMs) {
      Serial.println("\nIP address timeout");
      return false;
    }
    Serial.print(".");
    Serial.flush();
    delay(300);
    yield();
  }

  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  return true;
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
    body = "";
    body.reserve((size_t)contentLength);

    int remaining = contentLength;
    unsigned long start = millis();
    char tmp[128];

    while (remaining > 0 && (millis() - start) < READ_TIMEOUT_MS) {
      int avail = client.available();
      if (avail <= 0) {
        delay(1);
        yield();
        continue;
      }

      int toRead = remaining;
      if (toRead > (int)sizeof(tmp)) toRead = (int)sizeof(tmp);
      if (toRead > avail) toRead = avail;

      int got = client.readBytes(tmp, toRead);
      if (got <= 0) break;

      body.concat(String(tmp).substring(0, got));
      remaining -= got;
      yield();
    }
  } else {
    unsigned long start = millis();
    while (millis() - start < READ_TIMEOUT_MS) {
      while (client.available()) {
        body += (char)client.read();
      }
      if (body.length() > 0) break;
      delay(1);
      yield();
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
  if (v == 5) parking = true;
  switch (v) {
    case 0: return CASE_0;
    case 1: return CASE_1;
    case 2: return CASE_2;
    case 3: return CASE_3;
    case 4: return CASE_4;
    case 5: return CASE_1;
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

// BFS on graph nodes (MAX_GRAPH_NODES). Accepts graph indices for start/target.
// Produces detourPath[] of CaseState values (skips junctions).
void computeDetourPath(GraphNode startG, GraphNode targetG) {
    int parent[MAX_GRAPH_NODES];
    bool visited[MAX_GRAPH_NODES] = {false};
    std::queue<int> q;

    for (int i = 0; i < MAX_GRAPH_NODES; i++) parent[i] = -1;

    q.push((int)startG);
    visited[(int)startG] = true;

    bool pathFound = false;
    while (!q.empty()) {
        int u = q.front(); q.pop();
        if (u == (int)targetG) { pathFound = true; break; }

        for (int v = 0; v < MAX_GRAPH_NODES; v++) {
            // Only traverse if edge exists and not blocked
            if (adjacencyMatrix[u][v] && !obstacleMatrix[u][v] && !visited[v]) {
                visited[v] = true;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    if (!pathFound) {
        Serial.println("No detour path found (graph-level). Robot will stop or retry.");
        detourActive = false;
        detourLength = 0;
        return;
    }

    // Reconstruct graph path (target -> start)
    int graphPath[MAX_GRAPH_NODES];
    int cnt = 0;
    int node = (int)targetG;
    while (node != -1) {
        graphPath[cnt++] = node;
        node = parent[node];
    }

    // Reverse and compress into CaseState detourPath (skip junctions)
    detourLength = 0;
    for (int i = cnt - 1; i >= 0; i--) {
        int g = graphPath[i];
        if (isJunctionNode(g)) continue;           // skip junctions
        CaseState cs = graphToCase(g);
        // avoid including currentCase as first element
        if (detourLength == 0 && cs == currentCase) continue;
        // avoid duplicates (graph can produce repeated CaseStates)
        if (detourLength > 0 && detourPath[detourLength - 1] == cs) continue;
        detourPath[detourLength++] = cs;
    }

    // Finalize detour state
    if (detourLength == 0) {
        detourActive = false;
        detourIndex = 0;
    } else {
        detourActive = true;
        detourIndex = 0; // start with the first case in detourPath
    }

    // Debug print
    DBG_PRINT("Detour (case sequence): ");
    for (int i = 0; i < detourLength; i++) {
        DBG_PRINT(detourPath[i]);
        if (i < detourLength - 1) DBG_PRINT(" -> ");
    }
    DBG_PRINTLN("");
}

void driveUntilJunction() {
  // drive forward using line follow until a junction/node is detected
  while (!nodeDetected()) {
    readSensorsAndPrint();
    lineFollowStep();
    delay(5);
  }
  Brake(50);
}

void stopAndHalt() {
  Serial.println("Route complete. Stopping motors and halting.");
  Brake(0);
  while (true) {
    // Keep the MCU responsive for WiFi/RTOS housekeeping
    delay(50);
    yield();
  }
}
// Prefer the non-blocking ultrasonicTask() result where possible
float latestUltrasoundCm() {
  // If the reading is stale, treat as unknown/far
  if (millis() - latestDistanceMs > 200) return 999;
  return latestDistance;
}

float getDistance() {

    // settle sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);

    // trigger pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 18000); // 18ms = 3m max

    if (duration == 0)
        return 999; // no echo

    float distance = (duration * 0.0343) / 2.0;

    // ===== CRITICAL FILTER =====
    // reject floor reflections & noise
    if (distance < 12.0) return 999;

    return distance;
}

float readUltrasoundCm() {
    float sum = 0;
    int valid = 0;

    for (int i = 0; i < 5; i++) {
        float d = getDistance();
        if (d > 12 && d < 300) {
            sum += d;
            valid++;
        }
        delay(15);
    }

    if (valid == 0) return 999;
    return sum / valid;
}

void driveStraightUntilObstacle() {
  bool straightObstacle = false;
  bool leftObstacle = false;
  float distance;
  readSensorsAndPrint();
  while (!nodeDetected()) {
      readSensorsAndPrint();
      lineFollowStep();
  }

  parkingStart = millis();
  while(true) {
    elapsed = millis() - parkingStart;
    digitalWrite(motor1Phase, ACW);
    digitalWrite(motor2Phase, CW);
    analogWrite(motor1PWM, 99);
    analogWrite(motor2PWM, 100);
    delay(50);

    if (elapsed > 4000) {
      Serial.print("Drove for 4 seconds");
      Brake(0);
      parkingStart = millis();
      elapsed = 0;
      break;
    }
  }

    distance = latestUltrasoundCm();
    // ensure we have a recent reading before deciding
    if (millis() - latestDistanceMs > 200) {
      // force a quick update attempt
      ultrasonicTask();
      delay(10);
      ultrasonicTask();
      distance = latestUltrasoundCm();
    }
    if (distance < 55.0) {
      turnRight90();
      parkingStart = millis();
      while (true) {
        elapsed = millis() - parkingStart;
        digitalWrite(motor1Phase, ACW);
        digitalWrite(motor2Phase, CW);
        analogWrite(motor1PWM, 99);
        analogWrite(motor2PWM, 100);
        delay(50);
        if (elapsed >= 1200) {
          Brake(0);
          parkingStart = millis();
          elapsed = 0;
          break;
        }
      }
      turnLeft90();
      straightObstacle = true;
    }
     if (straightObstacle == false) {
      turnLeft20();
      distance = latestUltrasoundCm();
      // ensure we have a recent reading before deciding
      if (millis() - latestDistanceMs > 200) {
        ultrasonicTask();
        delay(10);
        ultrasonicTask();
        distance = latestUltrasoundCm();
      }
      if (distance < 60.0) {
        turnRight20();
        turnRight90();
        parkingStart = millis();
        while (true) {
          elapsed = millis() - parkingStart;
          digitalWrite(motor1Phase, ACW);
          digitalWrite(motor2Phase, CW);
          analogWrite(motor1PWM, 99);
          analogWrite(motor2PWM, 100);
          delay(50);
          if (elapsed >= 2000) {
            Brake(0);
            parkingStart = millis();
            elapsed = 0;
            break;
          }
        }
        turnLeft90();
        leftObstacle = true;
      } else {
        turnRight20();
      }
    } 
    if (straightObstacle == false && leftObstacle == false) {
      turnRight20();
      distance = latestUltrasoundCm();
      // ensure we have a recent reading before deciding
      if (millis() - latestDistanceMs > 200) {
        ultrasonicTask();
        delay(10);
        ultrasonicTask();
        distance = latestUltrasoundCm();
      }
      if (distance < 60.0) {
        turnLeft20();
        turnLeft90();
        parkingStart = millis();
        while (true) {
          elapsed = millis() - parkingStart;
          digitalWrite(motor1Phase, ACW);
          digitalWrite(motor2Phase, CW);
          analogWrite(motor1PWM, 99);
          analogWrite(motor2PWM, 100);
          delay(50);
          if (elapsed >= 2000) {
            Brake(0);
            parkingStart = millis();
            elapsed = 0;
            break;
          }
        }
        turnRight90();
      } else {
          turnLeft20();
      }
      }

    while (true) {
        // Keep ultrasonic updates flowing even during blocking parking loops
        ultrasonicTask();

        distance = latestUltrasoundCm();

        // If sensor is stale/unknown, be conservative: slow down
        bool stale = (millis() - latestDistanceMs > 200);

        Serial.print("Distance (cm): ");
        Serial.println(distance);

        // Stop earlier to avoid barreling into obstacles (tune 12–20cm)
        if (!stale && distance <= 15.0) {
            Brake(0);
            break;
        }

        // Drive straight (slow if stale)
        digitalWrite(motor1Phase, ACW);
        digitalWrite(motor2Phase, CW);
        if (stale) {
          analogWrite(motor1PWM, 60);
          analogWrite(motor2PWM, 60);
        } else {
          analogWrite(motor1PWM, 99);
          analogWrite(motor2PWM, 100);
        }

        delay(30); // smaller loop delay for faster reaction
        yield();
    }
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

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  delay(1000);
  if (!connectToWiFi()) {
    Serial.println("WiFi unavailable; continuing without server.");
  }

  // Try server with backoff, but do not block forever
  bool ok = false;
  for (int attempt = 0; attempt < 20; attempt++) {
    ok = connectServer();
    if (ok) break;
    delay(250);
    yield();
  }

  if (ok) {
    Serial.println("Connected to server.");

    bool routeFetched = false;
    for (int attempt = 0; attempt < 20; attempt++) {
      routeFetched = fetchRouteFromServer();
      if (routeFetched) break;
      Serial.println("Fetching route failed; retrying...");
      delay(250);
      yield();
    }

    // Route is now received and assigned to CASEs (or empty/finished)
    routeReady = routeFetched;
  } else {
    Serial.println("Server unavailable; route not ready.");
    routeReady = false;
  }
}

void loop() {
  ultrasonicTask();
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
    
    if (ignoreNextNode) {
    ignoreNextNode = false;
    nodeCounter = 0;
    return;   // skip logic once
    }

    nodeCounter++;
    Serial.print("Node hit: ");
    Serial.println(nodeCounter);
    handleNodeLogic();
  }

  // Start driving (line following) after route is ready. This will carry the car to the first node.
  if (!actionInProgress) {
    lineFollowStep();
  }
  if (!detourActive && startedRoute && routeIndex < routeLength) {

    // Rate-limit obstacle processing
    if (millis() - lastObstacleCheck >= OBSTACLE_CHECK_INTERVAL) {
      lastObstacleCheck = millis();

      if (newReading) {
        newReading = false;

        if (latestDistance < 20) {
          obstacleConfidence++;
        } else {
          obstacleConfidence = 0;
        }

        if (obstacleConfidence >= 3) {
          int startG = (int)caseToGraph(currentCase);
          int targetG = (int)caseToGraph(route[routeIndex]);
          int sharedJ = findSharedJunction(startG, targetG);

          if (sharedJ != -1) {
            obstacleMatrix[sharedJ][targetG] = true;
            obstacleMatrix[targetG][sharedJ] = true;
            DBG_PRINT("Blocked physical edge:");
            DBG_PRINT(" startG="); DBG_PRINT(startG);
            DBG_PRINT(" targetG="); DBG_PRINT(targetG);
            DBG_PRINT(" sharedJ="); DBG_PRINTLN(sharedJ);
          } else {
            obstacleMatrix[startG][targetG] = true;
            obstacleMatrix[targetG][startG] = true;
            DBG_PRINT("Blocked fallback edge: ");
            DBG_PRINT(startG);
            DBG_PRINT(" <-> ");
            DBG_PRINTLN(targetG);
          }

          Brake(50);
          flip180();
          driveUntilJunction();
          DBG_PRINT("Arrived back at physical junction (sharedJ): "); DBG_PRINTLN(sharedJ);
          Brake(50);

          // reset nodeCounter AFTER physically arriving at junction
          nodeCounter = 0;

          // compute detour starting from current physical junction
          GraphNode detourStart = (sharedJ != -1) ? (GraphNode)sharedJ : (GraphNode)startG;
          computeDetourPath(detourStart, (GraphNode)targetG);

          // Skip the first detour step if robot is already there
          if (detourActive && detourLength > 0 && graphToCase(detourStart) == detourPath[0]) {
            detourIndex = 1;
          } else {
            detourIndex = 0;
          }

          ignoreNextNode = false;  // reset flag; first detour node should be processed
          lastNodeTime = millis();

          DBG_PRINT("Detour ready. Index start at "); DBG_PRINTLN(detourIndex);
        }
      }
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

  CaseState nextCase;

  if (detourActive && detourIndex < detourLength) {
      nextCase = detourPath[detourIndex];
  } else {
      nextCase = route[routeIndex];
  }

  Serial.print("DBG handleNodeLogic: currentCase=");
  Serial.print(currentCase);
  Serial.print(" detourActive=");
  Serial.print(detourActive);
  Serial.print(" detourIndex=");
  Serial.print(detourIndex);
  Serial.print(" nextCase=");
  Serial.println(nextCase);


  //---------------------IF ROUTE IS 1 -> 5------------------------
  if (parking && currentCase == CASE_1 && nextCase == CASE_1) {
    Serial.println("Starting parking routine immediately");
    advanceCase(CASE_1);
    driveStraightUntilObstacle();
    sendArrivalToServer(5);
    finalPos = 5;
    sendFinishedToServer(finalPos);
    stopAndHalt();
    return;  // skip the rest of node logic
  }

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

    if (detourActive) {
        // Move to next detour step (we just entered the current detour element)
        detourIndex++;

        // End detour if we've consumed all detour steps OR if we've reached the original route target
        if (detourIndex >= detourLength || currentCase == route[routeIndex]) {
            detourActive = false;
            detourIndex = 0;
            detourLength = 0;
            // Completed the detour: advance the main route pointer
            routeIndex++;
        }
    } else {
        // Normal (non-detour) progress along original route
        routeIndex++;
    }

    continueForwardShort();

    if (routeIndex >= routeLength) {
        if (parking == true) {
            driveStraightUntilObstacle();
            sendArrivalToServer(5);
            finalPos = 5;
        } else {
            finalPos = (int)currentCase;
        }

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
  CaseState upcomingCase;

  if (detourActive) {
      if (detourIndex + 1 < detourLength)
          upcomingCase = detourPath[detourIndex + 1];
      else
          upcomingCase = currentCase;
  } else {
      upcomingCase = (routeIndex + 1 < routeLength) ? route[routeIndex + 1] : currentCase;
  }


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
        else if (parking == true) {
          if (lastCase != CASE_0 && lastCase != CASE_2) flip180();
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
  if (!detourActive) {
      sendArrivalToServer((int)currentCase);
  }
  else {
      if (currentCase == route[routeIndex]) {
          sendArrivalToServer((int)currentCase);
      }
  }

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

void turnRight20() {
  actionInProgress = true;
  spinRight(100);
  delay(166);   // ~20 degrees (tune if needed)
  Brake(50);
  actionInProgress = false;
}

void turnLeft20() {
  actionInProgress = true;
  spinLeft(100);
  delay(166);   // ~20 degrees (tune if needed)
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
