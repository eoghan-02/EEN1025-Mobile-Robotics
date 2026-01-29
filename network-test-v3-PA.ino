#include <WiFi.h>
char ssid[] = "Pele16Pro";
char password[] = "cursethis";
WiFiClient client;
char server[] = "3.250.38.184";
int port = 8000;

#define READ_TIMEOUT_MS 2000
int route[20];
int routeLength = 0;
int routeIndex = 0;

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
  Serial.println("Connected");
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
  if (!client.connect(server, port)) {
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

void setup() {
  Serial.begin(9600);
  delay(1000);
  connectToWiFi();
  bool check_connection = connectServer();
  while (check_connection == false) {
    check_connection = connectServer();
  }
  Serial.println("Connected to server.");
  String responseBody;
  int statusCode;
  bool routeFetched = false;
  while (!routeFetched) {
    client.println("GET /api/getRoute/pllk3098 HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Connection: keep-alive");
    client.println();
    Serial.println("Asking the server what my next stops are...");
    if (!readHttpResponse(statusCode, responseBody)) {
      Serial.println("Hmm, I didn't get a proper reply. I'll try again.");
      continue;
    }
    Serial.println("Server says: " + responseBody);
    Serial.println("HTTP status code: " + String(statusCode));

    if (statusCode == 200) {
      if (responseBody.startsWith("Finished") || responseBody.startsWith("Already Finished")) {
        Serial.println("Arrived at final destination.");
        return;
      }
      
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

        int position = posStr.toInt();
        // Serial.println("Heading to position " + String(position) + "...");
        //
        // String postBody("position=");
        // postBody += position;
        //
        // client.println("POST /api/arrived/pllk3098 HTTP/1.1");
        // client.println("Host: " + String(server));
        // client.println("Content-Type: application/x-www-form-urlencoded");
        // client.print("Content-Length: ");
        // client.println(postBody.length());
        // client.println();
        // client.println(postBody);
        //
        // Serial.println("Told the server I've arrived at " + String(position) + ". Taking a short breather...");
        // delay(5000);
        if (routeLength < 20) {
          route[routeLength] = position;
          routeLength++;
        }
      }
      routeFetched = true;
      Serial.println("That's the full route done.");
    } else {
      Serial.println("No OK, I'll try fetching the route again.");
      continue;
    }
  }
  Serial.println("All positions processed.");
}

void sendArrivalToServer(int position) {
  String postBody = "position=" + String(position);

  client.println("POST /api/arrived/pllk3098 HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println();
  client.println(postBody);

  Serial.println("Reported arrival at position " + String(position) + " to server.");
}

void loop() {
  if (routeIndex < routeLength) {
    int currentPosition = (int)route[routeIndex];

    Serial.println("Travelling to position " + String(currentPosition) + "...");

    // Simulate movement time (replace with real movement logic later)
    delay(5000);

    // Report arrival to server incrementally
    if (client.connected()) {
      sendArrivalToServer(currentPosition);
    } else {
      Serial.println("Lost connection to server before reporting arrival.");
    }

    routeIndex++;
  }
}