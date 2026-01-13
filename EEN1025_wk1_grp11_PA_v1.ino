int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;

// the setup routine runs once when you press reset:
void setup() {
Serial.begin(9600);
pinMode(motor1PWM, OUTPUT);
pinMode(motor1Phase, OUTPUT);
pinMode(motor2PWM, OUTPUT);
pinMode(motor2Phase, OUTPUT);
// To correct initial jerk of Motor2 on Power Up ↓↓↓
delay(100);
digitalWrite(motor1Phase, LOW);
analogWrite(motor1PWM, 180);
delay(125);
// ↑↑↑ Sensor could make this redundant
    }

// the loop routine runs over and over again continuously:
void loop() {

  delay(50);
  // Forward: Motor1 HIGH, Motor2 LOW
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 100); // set speed of motor within 1-255 (1 doesn't move at all)
  analogWrite(motor2PWM, 100);
  Serial.println("Forward"); // Display motor direction, for monitoring in Arduino
  delay(2000); //2 seconds

  // Backward: Motor1 LOW, Motor2 HIGH
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, 100);
  analogWrite(motor2PWM, 100);
  Serial.println("Backward");
  delay(2000);
    }
