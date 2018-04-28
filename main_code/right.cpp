#include "right.h"


void right::drive_motor(int lmotor, int rmotor) {
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

void right:: gyro_turn(int amount) {
  while (1) {
    imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float current = event.x();

    if (!is_turning) {
      turn_amount = amount;
      create_target(current);
      is_turning = true;
    }

    //   Serial.print(target);
    //   Serial.print(", ");
    //   Serial.println(current);

    if (!(current <= target-0.3 || current >= target+0.3) || (target < 0) || (target > 360)) {
      drive_motor(0, 0);
      delay(250);
      is_turning = false;
      break;
    }
    else {
      if (turn_amount > 0) {
        drive_motor(90, -90);
      }
      else {
        drive_motor(-90, 90);
      }
    }
  }
}

void right::create_target(float current) {
  target = current + turn_amount;

  if (target < 0) {
    target = 360 + target;
  }
  else if (target > 360) {
    target = target - 360;
  }

  if (target == 360) {
    target -= 0.3;
  }
}

void right::action(){
  //Serial.println("right");
 
    gyro_turn(90);
 
}
