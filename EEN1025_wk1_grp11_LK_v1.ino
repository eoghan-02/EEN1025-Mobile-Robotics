int motor1PWM = 37; //lhs motor
int motor1Phase = 38; 
int motor2PWM = 39; //RHS motor
int motor2Phase = 20;

// the setup routine runs once when you press reset:
void setup() {
Serial.begin(9600);
pinMode(motor1PWM, OUTPUT);
pinMode(motor1Phase, OUTPUT);
pinMode(motor2PWM, OUTPUT);
pinMode(motor2Phase, OUTPUT);
    }

// the loop routine runs over and over again continuously:
void loop() {

  digitalWrite(motor1Phase, HIGH); //forward
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100);
  Serial.println("Forward"); // Display motor direction
  delay(2000); //2 seconds

  digitalWrite(motor1Phase, LOW); //Backward
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, 100); // set speed of motor
  analogWrite(motor2PWM, 100);
  Serial.println("Backward"); // Display motor direction
  delay(2000); //2 seconds
    }

void MoveForward(int speed){

  digitalWrite(motor1Phase, HIGH); //forward
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  Serial.println("Forward"); // Display motor direction
}

void MoveBackward(int speed){

  digitalWrite(motor1Phase, LOW); //forward
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  Serial.println("Backwards"); // Display motor direction

}

void spinRight(int speed){

  digitalWrite(motor1Phase, HIGH ); //RHS motor reverse
  digitalWrite(motor2Phase, HIGH); //LHS motor forward 
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  Serial.println("Im Dizzy (RHS)"); // Display motor direction

}

void spinLeft(int speed){

  digitalWrite(motor1Phase, LOW); //RHS motor reverse
  digitalWrite(motor2Phase, LOW); //LHS motor forward
  analogWrite(motor1PWM, speed); // set speed of motor
  analogWrite(motor2PWM, speed);
  Serial.println("Im Dizzy (LHS)"); // Display motor direction
}

void Brake(){

  digitalWrite(motor1Phase, LOW); //RHS motor reverse
  digitalWrite(motor2Phase, LOW); //LHS motor forward
  analogWrite(motor1PWM, 0); // set speed of motor
  analogWrite(motor2PWM, 0);

}
