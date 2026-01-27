#include <WiFi.h>

char ssid[] = "iPhone";
char password[] = "haider1432";
char serverIP[] = "3.250.38.184";
int serverPort = 8000;
char teamID[] = "pllk3098";

#define BUFSIZE 512

void connectToWiFi() {
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nConnected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

String readResponse(WiFiClient &client) {
  String response = "";
  unsigned long startTime = millis();

  while (!client.available() && millis() - startTime < 5000) {
    delay(10);
  }

  startTime = millis();
  while (millis() - startTime < 5000) {
    while (client.available()) {
      char c = client.read();
      response += c;
      startTime = millis();
      if (response.length() >= BUFSIZE - 1) break;
    }
  }

  return response;
}

int getStatusCode(String &response) {
  String code = response.substring(9, 12);
  return code.toInt();
}

String getResponseBody(String &response) {
  int split = response.indexOf("\r\n\r\n");
  String body = response.substring(split + 4);
  body.trim(); // removes whitespace and line breaks
  return body;
}

void testConnections(int destination) {
  WiFiClient client;

  if (!client.connect(serverIP, serverPort)) {
    Serial.println("POST connection failed!");
    return;
  }

  String postBody = "position=" + String(destination);

  client.println("POST /api/arrived/" + String(teamID) + " HTTP/1.1");
  client.println("Host: " + String(serverIP));
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println();
  client.print(postBody);

  unsigned long timeout = millis() + 5000;
  while (!client.available() && millis() < timeout) {
    delay(10);
  }

  if (client.available()) {
    Serial.println("Server response:");
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      Serial.println(line);
    }
  } else {
    Serial.println("No response received (timeout).");
  }

  client.stop();
  Serial.println("POST connection closed.\n");
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  connectToWiFi();

  while (true) {
    WiFiClient client;

    if (!client.connect(serverIP, serverPort)) {
      Serial.println("GET connection failed!");
      delay(2000);
      continue;
    }

    client.println("GET /api/getRoute/" + String(teamID) + " HTTP/1.1");
    client.println("Host: " + String(serverIP));
    client.println("Connection: close");
    client.println();

    String response = readResponse(client);
    client.stop();

    Serial.println("Raw GET response: [" + response + "]");

    int statusCode = getStatusCode(response);
    if (statusCode != 200) {
      Serial.println("GET request failed with status code " + String(statusCode));
      delay(2000);
      continue;
    }

    String body = getResponseBody(response);
    body.trim();

    if (body.equals("Finished")) {
      Serial.println("All positions completed. Exiting loop.");
      break;
    }

    int destination = body.toInt();
    if (destination == 0 && !body.equals("0")) {
      Serial.println("Invalid destination received: " + body);
      delay(2000);
      continue;
    }

    Serial.print("Next destination: ");
    Serial.println(destination);

    testConnections(destination);
  }
}

void loop() {
  // empty, everything handled in setup
}