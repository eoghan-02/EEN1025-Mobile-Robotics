// Line sensor pins (left → right)
int AnalogValue[5] = {0, 0, 0, 0, 0};
int AnalogPin[5]   = {4, 5, 6, 7, 15};  // TCRT5000 outputs


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  readSensorsAndPrint();
}

void readSensorsAndPrint() {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    int inv = 4095 - AnalogValue[i];
    sum += inv;
    Serial.print(AnalogValue[i]);
    Serial.print("\t");
  }
  int inv_mean = sum/5;
Serial.print(inv_mean);
Serial.println();
}
