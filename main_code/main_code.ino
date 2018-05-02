#include "Arduino.h"
#include <QTRSensors.h>
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
#include "stack.h"
#include "actionsAbstract.h"
#include "forward.h"
#include "left.h"
#include "right.h"

#define L_MOTOR_A 32
#define L_MOTOR_B 33
#define R_MOTOR_A 25
#define R_MOTOR_B 26
#define FAN_MOTOR_A 4
#define FAN_MOTOR_B 2

#define FA_CHANNEL 0
#define FB_CHANNEL 1
#define LA_CHANNEL 2
#define LB_CHANNEL 3
#define RA_CHANNEL 4
#define RB_CHANNEL 5

#define ENC_CPR 2248 // number of counts per revolution
#define WHEEL_CIRCUM 8.65 // circumference of the wheel in inches

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

stack Stack ;

unsigned long aim_timeout;

int servo1_freq = 3500;
int servo2_freq = 2700; // 6000 is approx directly forward

DFRobotIRPosition IRcam;
int fire_x_pos;
int fire_y_pos;

long global_xpos = 0;
long global_ypos = 0;
long reverse_dist;
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
double Kp = 1, Ki = 0, Kd = 0.7;
float gtarget = 0;

enum mainStates {
  drive,
  detect,
  attack,
  backtrack,
  end
};

enum movingStates {
  find_wall,
  straight,
  turnRight,
  turnLeft,
  jump,
  mini_jump,
  reverse,
  cliff_turn
};

enum attackingStates {
  faceFlame,
  approach,
  aim,
  extinguish
};

enum backStates {
  move,
  all_stop
};

mainStates actions = drive;
movingStates movingActions = find_wall;
attackingStates attackingActions = faceFlame;
backStates backActions = move;

PID wallPID(&wall_in, &wall_out, &wall_setpoint, Kp, Ki, Kd, DIRECT);
PID gyroPID(&gyro_in, &gyro_out, &gyro_setpoint, Kp, Ki, Kd, DIRECT);

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

QTRSensorsAnalog qtra((unsigned char[]) {35}, 1);
unsigned int sensorValues[1];

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

  ledcSetup(FA_CHANNEL, 100, 8);
  ledcAttachPin(FAN_MOTOR_A, FA_CHANNEL);

  ledcSetup(FB_CHANNEL, 100, 8);
  ledcAttachPin(FAN_MOTOR_B, FB_CHANNEL);

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

  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro_setpoint = event.x();
  gtarget = event.x();
  // gyroPID.SetMode(AUTOMATIC);

  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }

  Stack.initializeStack();
}

void gyro_turn(int amount) {
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

void gyroFollow(float targetAngle){
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

void run_fan(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    ledcWrite(FA_CHANNEL, speed);
    ledcWrite(FB_CHANNEL, 0);
  }
  else if (speed < 0) {
    ledcWrite(FA_CHANNEL, 0);
    ledcWrite(FB_CHANNEL, -speed);
  }
  else {
    ledcWrite(FA_CHANNEL, 0);
    ledcWrite(FB_CHANNEL, 0);
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

  long avg_enc = r_enc.getPosition();//(r_enc.getPosition() + l_enc.getPosition()) / 2;
  global_xpos += avg_enc * cos(current*(M_PI/180));
  global_ypos += avg_enc * sin(current*(M_PI/180));

  current += 180;
  if (current >= 360) {
    current -= 360;
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(r_enc.getPosition());
  delay(1000);

  Stack.push(new forward(avg_enc, current));
  l_enc.resetPosition();
  r_enc.resetPosition();

  // Serial.print(global_xpos * cos(current*(M_PI/180)));
  // Serial.print(", ");
  // Serial.println(cos(current*(M_PI/180)));
}

void loop() {
  qtra.read(sensorValues);
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("X ");
  lcd.setCursor(3,1);
  lcd.print((global_xpos*WHEEL_CIRCUM) / ENC_CPR);
  lcd.setCursor(7,1);
  lcd.print("Y ");
  lcd.setCursor(9,1);
  lcd.print((global_ypos*WHEEL_CIRCUM) / ENC_CPR);
  switch (actions) {
    case drive:
      switch (movingActions) {
        case find_wall:
          if (frontDistanceToWall() < 8) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = turnRight;
          }
          else {
            PID_drive(100, 100);
          }

          break;
        case straight:
          if (frontDistanceToWall() >= 15 && leftDistanceToWall() >= 15) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = mini_jump;
          }
          else if (frontDistanceToWall() >= 8) {
            float dist = leftDistanceToWall();
            float wall_error = dist - wall_setpoint;
            gyroFollow(gtarget - (wall_error*2));

            // if (leftDistanceToWall() < 7.95) {
            //   drive_motor(110 + wall_out, 110);
            // }
            // else if (leftDistanceToWall() > 8) {
            //   drive_motor(110, 125);
            // }
            // else {
            //   drive_motor(110, 110);
            // }
          }
          else {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          if (sensorValues[0] > 4000) {
            drive_motor(0, 0);
            reverse_dist = 500;
            update_global_pos();
            movingActions = reverse;
          }
          actions = detect;
          break;
        case turnRight:
          gyro_turn(90);
          l_enc.resetPosition();
          r_enc.resetPosition();
          event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

          gyro_setpoint = event.x();
          gtarget = event.x();
          movingActions = straight;
          Stack.push(new left());
          break;
        case turnLeft:
          gyro_turn(-90);
          l_enc.resetPosition();
          r_enc.resetPosition();
          event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

          gyro_setpoint = event.x();
          gtarget = event.x();
          movingActions = jump;
          Stack.push(new right());
          break;
        case jump:
          if(abs(l_enc.getPosition()) >= ENC_CPR*1.2 && abs(r_enc.getPosition()) >= ENC_CPR*1.2) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = straight;
          }
          else {
            PID_drive(120, 120);
          }

          if (sensorValues[0] > 4000) {
            drive_motor(0, 0);
            reverse_dist = (l_enc.getPosition() + r_enc.getPosition()) / 2;
            update_global_pos();
            movingActions = reverse;
          }

          if (frontDistanceToWall() < wall_setpoint) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          break;
        case mini_jump:
          if(abs(l_enc.getPosition()) >= ENC_CPR*0.55 && abs(r_enc.getPosition()) >= ENC_CPR*0.55) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = turnLeft;
          }
          else {
            PID_drive(120, 120);
          }

          if (sensorValues[0] > 4000) {
            drive_motor(0, 0);
            reverse_dist = (l_enc.getPosition() + r_enc.getPosition()) / 2;
            update_global_pos();
            movingActions = reverse;
          }

          if (frontDistanceToWall() < wall_setpoint) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          break;
        case reverse:
          if (abs(l_enc.getPosition()) >= reverse_dist && abs(r_enc.getPosition()) >= reverse_dist) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = cliff_turn;
          }
          else {
            PID_drive(-120, -120);

          }

          break;
        case cliff_turn:
          gyro_turn(95);
          l_enc.resetPosition();
          r_enc.resetPosition();
          event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

          gyro_setpoint = event.x();
          gtarget = event.x();
          movingActions = jump;
          Stack.push(new left());
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
          lcd.setCursor(0,0);
          lcd.print("Flame Found");
          delay(3000);
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
          servo2_freq = 6200;

          gyro_turn(90);
          l_enc.resetPosition();
          r_enc.resetPosition();

          Stack.push(new left());

          gyro_setpoint = event.x();
          attackingActions = approach;
          break;
        case approach:
          if (frontDistanceToWall() >= 10) {
            IRcam.requestPosition();

            if (IRcam.available()) {
              fire_x_pos = IRcam.readY(0);
              fire_y_pos = IRcam.readX(0);

              // printResult();
            }

            if (fire_y_pos < 400) {
              servo1_freq -= 10;
              if (fire_y_pos < 150) {
                drive_motor(0, 0);
              }
              else {
                if (fire_x_pos > 350) {
                  PID_drive(100, 125);
                }
                else if (fire_x_pos < 250) {
                  PID_drive(125, 100);
                }
                else {
                  PID_drive(100, 100);
                }
              }
            }
            else if (fire_y_pos > 500) {
              servo1_freq += 10;
              if (fire_x_pos > 350) {
                PID_drive(100, 125);
              }
              else if (fire_x_pos < 250) {
                PID_drive(125, 100);
              }
              else {
                PID_drive(100, 100);
              }
            }
            else {
              if (fire_x_pos > 350) {
                PID_drive(100, 125);
              }
              else if (fire_x_pos < 250) {
                PID_drive(125, 100);
              }
              else {
                PID_drive(100, 100);
              }
            }

            // if (fire_x_pos > 350) {
            //   servo2_freq += 3;
            // }
            // if (fire_x_pos < 250) {
            //   servo2_freq -= 3;
            // }

            ledcWrite(SERV1_CHANNEL, servo1_freq);
            ledcWrite(SERV2_CHANNEL, servo2_freq);
          }
          else {
            drive_motor(0, 0);
            update_global_pos();
            aim_timeout = millis();
            attackingActions = aim;
          }
          break;
        case aim:
          IRcam.requestPosition();
          if (IRcam.available()) {
            fire_x_pos = IRcam.readY(0);
            fire_y_pos = IRcam.readX(0);

            // printResult();
          }

          if (fire_x_pos != 1023 && fire_y_pos != 1023) {
            if (fire_x_pos > 350) {
              servo2_freq += 5;
            }
            if (fire_x_pos < 250) {
              servo2_freq -= 5;
            }

            if (fire_y_pos > 350) {
              servo1_freq += 5;
            }
            if (fire_y_pos < 250) {
              servo1_freq -= 5;
            }
          }

          if (fire_x_pos < 450 && fire_x_pos > 50 && fire_y_pos < 450 && fire_y_pos > 150) {
            attackingActions = extinguish;
          }
          if (millis() - aim_timeout >= 5000) {
            attackingActions = extinguish;
          }

          ledcWrite(SERV1_CHANNEL, servo1_freq);
          ledcWrite(SERV2_CHANNEL, servo2_freq);

          break;
        case extinguish:
          // do some angle magic stuff to get z-coord of flame

          run_fan(255);
          delay(20000);
          lcd.setCursor(0,0);
          lcd.print("Flame Blown");
          IRcam.requestPosition();
          if (IRcam.available()) {
            fire_x_pos = IRcam.readY(0);
            fire_y_pos = IRcam.readX(0);

            // printResult();
          }
          delay(5000);

          if (fire_x_pos == 1023 && fire_y_pos == 1023) {
            run_fan(0);

            global_xpos += 10 * cos(event.x()*(M_PI/180));
            global_ypos += 10 * sin(event.x()*(M_PI/180));

            actions = backtrack;
          }
          break;
      }
      break;
    case backtrack:
      gyro_turn(180);
      Stack.action();
      drive_motor(0, 0);
      actions = end;

      break;
      // switch (backActions) {
      //   case move:
      //     PID_drive(-120, -120);
      //     delay(2000);
      //     drive_motor(0,0);
      //     gyro_turn(180);
      //     backActions = all_stop;
      //     break;
      //   case all_stop:
      //     drive_motor(0, 0);
      //     break;
      // }
      // break;
      case end:
        lcd.setCursor(0, 0);
        lcd.print("Died");
        drive_motor(0, 0);
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

void cliff_isr() {
  movingActions = reverse;
}
