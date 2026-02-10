int Red1 = 4;
int Blue1 = 1;
int speaker = 40;
int ledCount = 0;


unsigned long previousMillis = 0;
unsigned long interval = 100;  // 50 ms
bool RedledState = LOW;
bool BlueledState = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(Red1, OUTPUT);
  pinMode(Blue1,OUTPUT);
}

void loop() {
  flashing();
  ledcWrite(speaker,255);
}

void flashing() {
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




