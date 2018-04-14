#define FAN_MOTOR_A 3
#define FAN_MOTOR_B 5

void setup() {
  // put your setup code here, to run once:
  pinMode(FAN_MOTOR_A, OUTPUT);
  pinMode(FAN_MOTOR_B, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  run_fan(255);
  // delay(10000);

  // run_fan(0);
  // delay(15000);
}

void run_fan(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(FAN_MOTOR_A, speed);
    analogWrite(FAN_MOTOR_B, 0);
  }
  else if (speed < 0) {
    analogWrite(FAN_MOTOR_A, 0);
    analogWrite(FAN_MOTOR_B, speed);
  }
  else {
    analogWrite(FAN_MOTOR_A, 0);
    analogWrite(FAN_MOTOR_B, 0);
  }
}
