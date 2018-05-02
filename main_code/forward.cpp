#include "forward.h"

//forward :: forward(){

//}
bool forward::lenc_trigger = false;
bool forward::renc_trigger = false;

forward :: forward(long dist, float angle){
  travelDistance = dist;
  gtarget = angle;

  // lcd.begin(21, 22);                      // initialize the lcd
  // lcd.backlight();
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print(travelDistance);
  // delay(1000);
}

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

// void forward::lenc_isr1() {
//   // forward::lenc_trigger = true;
//   // l_enc1.loop();
//   }
// }
//
// void forward::renc_isr1() {
//   // forward::renc_trigger = true;
//   // r_enc1.loop();
// }

double forward::frontDistanceToWall() {
  // Clears the trigPin
  digitalWrite(fronttrigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(fronttrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(fronttrigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(frontechoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  delay(5);
  return distance;
}

double forward::rightDistanceToWall() {
  // Clears the trigPin
  digitalWrite(righttrigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(righttrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(righttrigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(rightechoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  delay(5);
  return distance;
}

double forward::leftDistanceToWall() {
  // Clears the trigPin
  digitalWrite(lefttrigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(lefttrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(lefttrigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(leftechoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  delay(5);
  return distance;
}

void forward::gyroFollow(float targetAngle){
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float current = event.x();
  if (current > 180) {
    current -= 360;
  }
  if (targetAngle > 180) {
    targetAngle -= 360;
  }

  float error = Kp*(targetAngle-current);
  drive_motor(100+error,100-error);
}

void forward :: action(){
  //Serial.println("forward");
   lcd.begin(21, 22);                      // initialize the lcd
   lcd.backlight();
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print(gtarget);

  lenc1 = digitalRead(ENC_LA);
  lenc2 = digitalRead(ENC_LB);
  renc1 = digitalRead(ENC_RA);
  renc2 = digitalRead(ENC_RB);

  l_enc1.resetPosition();
  r_enc1.resetPosition();
  while (1) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(l_enc1.getPosition());

    if (lenc1 != digitalRead(ENC_LA)) {
      lenc1 = digitalRead(ENC_LA);
      l_enc1.loop();
    }
    else if (lenc2 != digitalRead(ENC_LB)) {
      lenc2 = digitalRead(ENC_LB);
      l_enc1.loop();
    }
    if (renc1 != digitalRead(ENC_RA)) {
      renc1 = digitalRead(ENC_RA);
      r_enc1.loop();
    }
    else if (renc2 != digitalRead(ENC_RB)) {
      renc2 = digitalRead(ENC_RB);
      r_enc1.loop();
    }

    // lenc_isr1();
    // renc_isr1();
    // if (forward::lenc_trigger) {
    //   l_enc1.loop();
    //   forward::lenc_trigger = false;
    // }
    // if (forward::renc_trigger) {
    //   r_enc1.loop();
    //   forward::renc_trigger = false;
    // }

    if (frontDistanceToWall() < 8 || (abs(l_enc1.getPosition()) > travelDistance && abs(r_enc1.getPosition()) > travelDistance)) {
      drive_motor(0, 0);
      break;
    }
    if (rightDistanceToWall() < 8) {
      float wall_error = rightDistanceToWall() - wall_setpoint;
      gyroFollow(gtarget + (wall_error*2));
    }
    else if (leftDistanceToWall() < 8) {
      float wall_error = leftDistanceToWall() - wall_setpoint;
      gyroFollow(gtarget - (wall_error*2));
    }
    else {
      gyroFollow(gtarget);
    }
  }

  delay(5000);
}
