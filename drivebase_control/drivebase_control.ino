#include <ESPRotary.h>

#define L_MOTOR_A 32
#define L_MOTOR_B 33
#define R_MOTOR_A 25
#define R_MOTOR_B 26

#define LA_CHANNEL 2
#define LB_CHANNEL 3
#define RA_CHANNEL 4
#define RB_CHANNEL 5

#define ENC_CPR 2248 // number of counts per revolution
#define WHEEL_CIRCUM 8.5 // circumference of the wheel in inches

#define ENC_LA 16
#define ENC_LB 17
#define ENC_RA 13
#define ENC_RB 14


ESPRotary l_enc = ESPRotary(ENC_LA, ENC_LB, 1);
ESPRotary r_enc = ESPRotary(ENC_RA, ENC_RB, 1);

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

  attachInterrupt(digitalPinToInterrupt(ENC_LA), lenc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB), lenc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), renc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB), renc_isr, CHANGE);

  l_enc.resetPosition();
  r_enc.resetPosition();
}

void loop() {
//  l_enc.loop();
//  r_enc.loop();
  Serial.print("Left = ");
  Serial.print(l_enc.getPosition());
  Serial.print(", Right = ");
  Serial.println(r_enc.getPosition());

  if(abs(l_enc.getPosition()) > ENC_CPR*3 && abs(r_enc.getPosition()) > ENC_CPR*3) {
    drive_motor(0,0);
  }
  else {
    drive_motor(100, 100);
  }
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

void lenc_isr() {
  l_enc.loop();
}

void renc_isr() {
  r_enc.loop();
}
