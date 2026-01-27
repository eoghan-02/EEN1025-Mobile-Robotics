#include <Wire.h>
#include "HUSKYLENS.h"

HUSKYLENS huskylens;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(8, 9);

  while (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens not found on I2C...");
    delay(1000);
  }

  Serial.println("HuskyLens connected!");

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
  Serial.println("Object Tracking mode set.");
  Serial.println("Press LEARN on HuskyLens to track an object.");
}

void loop() {
  if (!huskylens.request()) return;

  if (huskylens.count() == 0) {
    Serial.println("No object.");
    delay(50);
    return;
  }

  while (huskylens.available()) {
    HUSKYLENSResult r = huskylens.read();
    if (r.command == COMMAND_RETURN_BLOCK) {
      Serial.print("x=");
      Serial.print(r.xCenter);
      Serial.print(" y=");
      Serial.print(r.yCenter);
      Serial.print(" w=");
      Serial.print(r.width);
      Serial.print(" h=");
      Serial.print(r.height);
      Serial.print(" id=");
      Serial.println(r.ID);
    }
  }

  delay(50);
}


