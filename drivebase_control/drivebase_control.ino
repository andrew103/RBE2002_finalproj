#include <ESPRotary.h>

#define L_MOTOR_A 32
#define L_MOTOR_B 33
#define R_MOTOR_A 25
#define R_MOTOR_B 26

#define LA_CHANNEL 2
#define LB_CHANNEL 3
#define RA_CHANNEL 4
#define RB_CHANNEL 5

#define ENC_CPR 48 // number of holes on encoder disk
#define WHEEL_CIRCUM 8.5 // circumference of the wheel in inches

#define ENC_LA 16
#define ENC_LB 17
#define ENC_RA 13
#define ENC_RB 14


ESPRotary l_enc = ESPRotary(ENC_LA, ENC_LB);
ESPRotary r_enc = ESPRotary(ENC_RA, ENC_RB);

void setup() {
  Serial.begin(115200);

  ledcSetup(LA_CHANNEL, 100, 8);
  ledcAttachPin(L_MOTOR_A, LA_CHANNEL);

  ledcSetup(LB_CHANNEL, 100, 8);
  ledcAttachPin(L_MOTOR_B, LB_CHANNEL);

  ledcSetup(RA_CHANNEL, 100, 8);
  ledcAttachPin(R_MOTOR_A, RA_CHANNEL);

  ledcSetup(RB_CHANNEL, 100, 8);
  ledcAttachPin(R_MOTOR_B, RB_CHANNEL);

  l_enc.resetPosition();
  r_enc.resetPosition();
}

void loop() {
  l_enc.loop();
  r_enc.loop();
  Serial.print("Left = ");
  Serial.print(l_enc.getPosition());
  Serial.print(", Right = ");
  Serial.println(r_enc.getPosition());

  drive_motor(100, 100);
  // delay(2000);
  //
  // drive_motor(200, -200);
  // delay(2000);
  //
  // drive_motor(-200, 200);
  // delay(2000);
}

void drive_motor(int lmotor, int rmotor) {
  lmotor = constrain(lmotor, -255, 255);
  rmotor = constrain(rmotor, -255, 255);

  if (lmotor > 0) {
    ledcWrite(LA_CHANNEL, lmotor);
    ledcWrite(LB_CHANNEL, 0);
  }
  else {
    ledcWrite(LA_CHANNEL, 0);
    ledcWrite(LB_CHANNEL, -lmotor);
  }

  if (rmotor > 0) {
    ledcWrite(RA_CHANNEL, rmotor);
    ledcWrite(RB_CHANNEL, 0);
  }
  else {
    ledcWrite(RA_CHANNEL, 0);
    ledcWrite(RB_CHANNEL, -rmotor);
  }
}
