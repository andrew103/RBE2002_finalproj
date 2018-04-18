#define FAN_MOTOR_A 2
#define FAN_MOTOR_B 4
#define A_CHANNEL 0
#define B_CHANNEL 1

void setup() {
  ledcSetup(A_CHANNEL, 100, 8);
  ledcAttachPin(FAN_MOTOR_A, A_CHANNEL);

  ledcSetup(B_CHANNEL, 100, 8);
  ledcAttachPin(FAN_MOTOR_B, B_CHANNEL);

  Serial.begin(115200);
}

void loop() {
  run_fan(255);
  // delay(10000);

  // run_fan(0);
  // delay(15000);
}

void run_fan(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    ledcWrite(A_CHANNEL, speed);
    ledcWrite(B_CHANNEL, 0);
  }
  else if (speed < 0) {
    ledcWrite(A_CHANNEL, 0);
    ledcWrite(B_CHANNEL, speed);
  }
  else {
    ledcWrite(A_CHANNEL, 0);
    ledcWrite(B_CHANNEL, 0);
  }
}
