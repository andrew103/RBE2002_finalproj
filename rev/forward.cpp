#include "forward.h"


void forward :: drive_motor(int lmotor, int rmotor) {
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

void forward::lenc_isr() {
  l_enc.loop();
}

void forward::renc_isr() {
  r_enc.loop();
}

void forward :: action(){
  //Serial.println("forward");
  while(1){
   if(abs(l_enc.getPosition()) > ENC_CPR*10 && abs(r_enc.getPosition()) > ENC_CPR*10) {
    drive_motor(0,0);
    break;
  }
  else {
    drive_motor(100, 100);
  }
  }
}

