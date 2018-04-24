#include "Arduino.h"
#include <PID_v1.h>
#include <Wire.h>
#include <ESPRotary.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"
#include "DFRobotIRPosition.h"
#include "esp32-hal-ledc.h"

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

#define SERVO_1 15
#define SERVO_2 23
#define SERV1_CHANNEL 6
#define SERV2_CHANNEL 7

#define COUNT_LOW_1 1450
#define COUNT_HIGH_1 4200
#define COUNT_LOW_2 1450
#define COUNT_HIGH_2 8000

#define TIMER_WIDTH 16

int servo1_freq = 3500;
int servo2_freq = 2500; // 6000 is approx directly forward

DFRobotIRPosition IRcam;
int fire_x_pos;
int fire_y_pos;

long global_xpos = 0;
long global_ypos = 0;
ESPRotary l_enc = ESPRotary(ENC_LA, ENC_LB, 1);
ESPRotary r_enc = ESPRotary(ENC_RA, ENC_RB, 1);

float target = -1.0;
int turn_amount;
bool is_turning = false;
Adafruit_BNO055 bno = Adafruit_BNO055();

// defines pins numbers
const int fronttrigPin = 5;
const int frontechoPin = 36;
const int lefttrigPin = 18;
const int leftechoPin = 39;
const int righttrigPin = 19;
const int rightechoPin = 34;

// defines variables
long duration;
double distance;
double wall_setpoint, wall_in, wall_out;
double gyro_setpoint, gyro_in, gyro_out;
double Kp = 2.5, Ki = 0, Kd = 0.65;

enum mainStates {
  drive,
  detect,
  attack,
  backtrack
};

enum movingStates {
  forward,
  turnRight,
  turnLeft,
  jump,
  mini_jump
};

enum attackingStates {
  faceFlame,
  approach,
  extinguish
};

mainStates actions = drive;
movingStates movingActions = forward;
attackingStates attackingActions = faceFlame;

PID wallPID(&wall_in, &wall_out, &wall_setpoint, Kp, Ki, Kd, DIRECT);
PID gyroPID(&gyro_in, &gyro_out, &gyro_setpoint, Kp, Ki, Kd, DIRECT);

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//sets up all the pins and library functions
void setup() {
  pinMode(fronttrigPin, OUTPUT); // Sets the fronttrigPin as an Output
  pinMode(frontechoPin, INPUT); // Sets the frontechoPin as an Input
  pinMode(lefttrigPin, OUTPUT); // Sets the lefttrigPin as an Output
  pinMode(leftechoPin, INPUT); // Sets the leftechoPin as an Input
  pinMode(righttrigPin, OUTPUT); // Sets the righttrigPin as an Output
  pinMode(rightechoPin, INPUT); // Sets the rightechoPin as an Input
  Serial.begin(115200); // Starts the serial communication

  wall_setpoint = 8;
  wallPID.SetMode(AUTOMATIC);

  ledcSetup(LA_CHANNEL, 100, 8);
  ledcAttachPin(L_MOTOR_A, LA_CHANNEL);

  ledcSetup(LB_CHANNEL, 100, 8);
  ledcAttachPin(L_MOTOR_B, LB_CHANNEL);

  ledcSetup(RA_CHANNEL, 100, 8);
  ledcAttachPin(R_MOTOR_A, RA_CHANNEL);

  ledcSetup(RB_CHANNEL, 100, 8);
  ledcAttachPin(R_MOTOR_B, RB_CHANNEL);

  ledcSetup(SERV1_CHANNEL, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(SERVO_1, SERV1_CHANNEL);   // GPIO 23 assigned to channel 1

  ledcSetup(SERV2_CHANNEL, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(SERVO_2, SERV2_CHANNEL);   // GPIO 23 assigned to channel 1

  IRcam.begin();

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

  attachInterrupt(digitalPinToInterrupt(ENC_LA), lenc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB), lenc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), renc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB), renc_isr, CHANGE);

  l_enc.resetPosition();
  r_enc.resetPosition();

  lcd.begin(21, 22);                      // initialize the lcd
  lcd.backlight();
}

void gyro_turn(int amount) {
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

  if (!(current <= target-0.5 || current >= target+0.5) || (target < 0) || (target > 360)) {
    drive_motor(0, 0);
    delay(250);
    is_turning = false;
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

void create_target(float current) {
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

void PID_drive(int lmotor, int rmotor) {
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro_in = event.x();
  gyroPID.Compute();

  double opp_set = gyro_setpoint + 180;
  if (opp_set >= 360) {
    opp_set -= 360;
  }

  if (gyro_in < gyro_setpoint || gyro_in > opp_set) {
    drive_motor(lmotor + gyro_out, rmotor);
  }
  else if (gyro_in > gyro_setpoint || gyro_in < opp_set) {
    drive_motor(lmotor, rmotor + gyro_out);
  }
  else {
    drive_motor(lmotor, rmotor);
  }
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

double frontDistanceToWall() {
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

double leftDistanceToWall() {
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

double rightDistanceToWall() {
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

void update_global_pos() {
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float current = event.x();

  int avg_enc = (r_enc.getPosition() + l_enc.getPosition()) / 2;
  global_xpos += avg_enc * cos(current*(M_PI/180));
  global_ypos += avg_enc * sin(current*(M_PI/180));

  l_enc.resetPosition();
  r_enc.resetPosition();

  // Serial.print(global_xpos * cos(current*(M_PI/180)));
  // Serial.print(", ");
  // Serial.println(cos(current*(M_PI/180)));
}

void loop() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print((global_xpos*WHEEL_CIRCUM) / ENC_CPR);
  lcd.setCursor(0,1);
  lcd.print((global_ypos*WHEEL_CIRCUM) / ENC_CPR);
  switch (actions) {
    case drive:
      switch (movingActions) {
        case forward:
          if (frontDistanceToWall() >= 15 && leftDistanceToWall() >= 15) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = mini_jump;
          }
          else if (frontDistanceToWall() >= 8) {
            wall_in = leftDistanceToWall();
            wallPID.Compute();

            if (leftDistanceToWall() < 7.95) {
              drive_motor(105 + wall_out, 100);
            }
            else if (leftDistanceToWall() > 8) {
              drive_motor(100, 100 + wall_out);
            }

            else {
              drive_motor(100, 100);
            }
          }
          else {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          actions = detect;
          break;
        case turnRight:
          gyro_turn(90);
          if (!is_turning) {
            drive_motor(0, 0);
            l_enc.resetPosition();
            r_enc.resetPosition();
            movingActions = forward;
          }

          break;
        case turnLeft:
          gyro_turn(-90);
          if(!is_turning) {
            drive_motor(0, 0);
            l_enc.resetPosition();
            r_enc.resetPosition();
            movingActions = jump;
          }

          break;
        case jump:
          if(abs(l_enc.getPosition()) >= ENC_CPR && abs(r_enc.getPosition()) >= ENC_CPR) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = forward;
          }
          else {
            PID_drive(100, 100);
          }

          break;
        case mini_jump:
          if(abs(l_enc.getPosition()) >= ENC_CPR*0.65 && abs(r_enc.getPosition()) >= ENC_CPR*0.65) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = turnLeft;
          }
          else {
            PID_drive(100, 100);
          }

          break;
      }
      break;
    case detect:
      ledcWrite(SERV1_CHANNEL, servo1_freq);
      ledcWrite(SERV2_CHANNEL, servo2_freq);

      IRcam.requestPosition();
      if (IRcam.available()) {
        fire_x_pos = IRcam.readY(0);
        fire_y_pos = IRcam.readX(0);

        // printResult();
        if (fire_x_pos < 350 && fire_x_pos > 250) {
          drive_motor(0, 0);
          update_global_pos();
          actions = attack;
        }
        else {
          actions = drive;
        }
      }
      break;
    case attack:
      switch (attackingActions) {
        case faceFlame:
          break;
        case approach:
          break;
        case extinguish:
          actions = backtrack;
          break;
      }
      break;
    case backtrack:
      break;
  }
}

void printResult() {
  Serial.print(fire_x_pos);
  Serial.print(",");

  Serial.print(fire_y_pos);
  Serial.println(";");
}

void lenc_isr() {
  l_enc.loop();
}

void renc_isr() {
  r_enc.loop();
}
