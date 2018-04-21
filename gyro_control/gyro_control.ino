#include <Wire.h>
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"

#define L_MOTOR_A 32
#define L_MOTOR_B 33
#define R_MOTOR_A 25
#define R_MOTOR_B 26

#define LA_CHANNEL 2
#define LB_CHANNEL 3
#define RA_CHANNEL 4
#define RB_CHANNEL 5

float target;
Adafruit_BNO055 bno = Adafruit_BNO055();

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

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);
}

void loop() {
//  sensors_event_t event;
//  bno.getEvent(&event);
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float start = event.x();
  int turn_amount = 180;
  target = start + turn_amount;
  if (target < 0) {
    target = 360 + target;
  }
  else if (target > 360) {
    target = target - 360;
  }

  while (1) {
    //sensors_event_t event;
    //bno.getEvent(&event);
    imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print(target);
    Serial.print(", ");
    Serial.println(event.x());

    if (!(event.x() <= target-0.3 || event.x() >= target+0.3)) {
      break;
    }

    if (turn_amount > 0) {
      drive_motor(80, -80);
    }
    else {
      drive_motor(-80, 80);
    }
  }
  drive_motor(0,0);
  delay(1000);
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
