// Import all necessary libraries for sensors, calculations, etc.
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

// Declare constants such as pins, channels, etc.
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

#define ENC_CPR 2248 // number of counts per revolution of the encoder
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

// Define ultrasonic sensor pins
const int fronttrigPin = 5;
const int frontechoPin = 36;
const int lefttrigPin = 18;
const int leftechoPin = 39;
const int righttrigPin = 19;
const int rightechoPin = 34;

long duration;
double distance;
double wall_setpoint, wall_in, wall_out;
double gyro_setpoint, gyro_in, gyro_out;
double Kp = 1, Ki = 0, Kd = 0.7;
float gtarget = 0;

// State machine that controls the overall operation of the robot
enum mainStates {
  drive,
  detect,
  attack,
  backtrack,
  end
};

// State machine that controls the driving operations of the robot
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

// State machine that takes care of extinguishing the flame
enum attackingStates {
  faceFlame,
  approach,
  aim,
  extinguish
};

// Initialize state machines
mainStates actions = drive;
movingStates movingActions = find_wall;
attackingStates attackingActions = faceFlame;

// Initialize PID instances
PID wallPID(&wall_in, &wall_out, &wall_setpoint, Kp, Ki, Kd, DIRECT);
PID gyroPID(&gyro_in, &gyro_out, &gyro_setpoint, Kp, Ki, Kd, DIRECT);

// Initialize LCD instance
LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Initialize reflectance sensor instance (cliff detection)
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

  ledcSetup(SERV1_CHANNEL, 50, TIMER_WIDTH);
  ledcAttachPin(SERVO_1, SERV1_CHANNEL);

  ledcSetup(SERV2_CHANNEL, 50, TIMER_WIDTH);
  ledcAttachPin(SERVO_2, SERV2_CHANNEL);

  IRcam.begin();

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

  l_enc.resetPosition();
  r_enc.resetPosition();

  lcd.begin(21, 22);
  lcd.backlight();

  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro_setpoint = event.x();
  gtarget = event.x();
  // gyroPID.SetMode(AUTOMATIC);

  // Calibrate the reflectance sensor
  for (int i = 0; i < 400; i++)
  {
    qtra.calibrate();
  }

  // Initialize the stack instance
  Stack.initializeStack();

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENC_LA), lenc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB), lenc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), renc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB), renc_isr, CHANGE);
}

// Handles turning the robot a given number of degrees to the left/right
void gyro_turn(int amount) {
  while (1) {
    // update current heading
    imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float current = event.x();

    if (!is_turning) {
      turn_amount = amount;
      create_target(current);
      is_turning = true;
    }

    // land within an acceptable error range
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

// Helper function for gyro_turn that creates the target angle based on the
// given input and current heading
void create_target(float current) {
  target = current + turn_amount;

  // adjusts for values outside of the 0-360 range
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

// Drives in a straight line maintaining a given heading (proportional control)
void gyroFollow(float targetAngle){
  // get current heading
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float current = event.x();

  // adjust givens to be in the -180 to 180 range
  if (current > 180) {
    current -= 360;
  }
  if (targetAngle > 180) {
    targetAngle -= 360;
  }

  float error = Kp*(targetAngle-current);
  drive_motor(100+error,100-error);
}

// Drives in a straight line using PID with the gyro
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

// Takes care of running the motors at the given speeds for each side
void drive_motor(int lmotor, int rmotor) {
  // constrain the input values to within the required range of operation for
  // the motor controller
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

// Takes care of running the fan at the given speed
void run_fan(int speed) {
  // constrain the input values to within the required range of operation for
  // the motor controller
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

  delay(5);
  return distance;
}

// Takes care of updating the global position of the robot with respect to its
// starting position after going straight then stopping
void update_global_pos() {
  // get current heading
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float current = event.x();

  // get an avg encoder reading from the two encoders and use trig with the
  // heading to calculate X,Y position change
  long avg_enc = (r_enc.getPosition() + l_enc.getPosition()) / 2;
  global_xpos += avg_enc * cos(current*(M_PI/180));
  global_ypos += avg_enc * sin(current*(M_PI/180));

  // current += 180;
  // if (current >= 360) {
  //   current -= 360;
  // }

  // Stack.push(new forward(avg_enc, current));

  // zero the encoder values for the next operation
  l_enc.resetPosition();
  r_enc.resetPosition();
}

void loop() {
  // get current reflectance sensor value and heading
  qtra.read(sensorValues);
  imu::Vector<3> event = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // print global coordinates on the LCD display
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("X ");
  lcd.setCursor(3,1);
  lcd.print((global_xpos*WHEEL_CIRCUM) / ENC_CPR); // calculate distance in inches
  lcd.setCursor(7,1);
  lcd.print("Y ");
  lcd.setCursor(9,1);
  lcd.print((global_ypos*WHEEL_CIRCUM) / ENC_CPR); // calculate distance in inches
  switch (actions) {
    case drive:
      switch (movingActions) {
        // moves forward from starting position until wall is encountered
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
        // handles left wall following and all exit conditions
        case straight:
          // wall ended
          if (frontDistanceToWall() >= 15 && leftDistanceToWall() >= 15) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = mini_jump;
          }
          // still a left wall
          else if (frontDistanceToWall() >= 8) {
            float dist = leftDistanceToWall();
            float wall_error = dist - wall_setpoint;
            gyroFollow(gtarget - (wall_error*2));
          }
          // front wall detected
          else {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          // cliff detected
          if (sensorValues[0] > 4000) {
            drive_motor(0, 0);
            reverse_dist = 500;
            update_global_pos();
            movingActions = reverse;
          }
          actions = detect; // switch to detection mode to look for flame
          break;
        // 90 degree turn to the right
        case turnRight:
          gyro_turn(90); // execute turn
          l_enc.resetPosition(); // zero left encoder
          r_enc.resetPosition(); // zero right encoder
          event = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // get current heading

          // update setpoints
          gyro_setpoint = event.x();
          gtarget = event.x();
          movingActions = straight;
          Stack.push(new left());
          break;
        // 90 degree turn to the right
        case turnLeft:
          gyro_turn(-90); // execute turn
          l_enc.resetPosition(); // zero left encoder
          r_enc.resetPosition(); // zero right encoder
          event = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // get current heading

          // update setpoints
          gyro_setpoint = event.x();
          gtarget = event.x();
          movingActions = jump;
          Stack.push(new right());
          break;
        // move forward some distance
        case jump:
          // check if current encoder values have met/exceeded target values
          if(abs(l_enc.getPosition()) >= ENC_CPR*1.2 && abs(r_enc.getPosition()) >= ENC_CPR*1.2) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = straight;
          }
          else {
            PID_drive(120, 120);
          }

          // cliff detected
          if (sensorValues[0] > 4000) {
            drive_motor(0, 0);
            reverse_dist = (l_enc.getPosition() + r_enc.getPosition()) / 2;
            update_global_pos();
            movingActions = reverse;
          }

          // front wall detected
          if (frontDistanceToWall() < wall_setpoint) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          break;
        // move forward a small distance
        case mini_jump:
          // check if current encoder values have met/exceeded target values
          if(abs(l_enc.getPosition()) >= ENC_CPR*0.55 && abs(r_enc.getPosition()) >= ENC_CPR*0.55) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = turnLeft;
          }
          else {
            PID_drive(120, 120);
          }

          // cliff detected
          if (sensorValues[0] > 4000) {
            drive_motor(0, 0);
            reverse_dist = (l_enc.getPosition() + r_enc.getPosition()) / 2;
            update_global_pos();
            movingActions = reverse;
          }

          // front wall detected
          if (frontDistanceToWall() < wall_setpoint) {
            drive_motor(0, 0);
            update_global_pos();
            movingActions = turnRight;
          }

          break;
        // drive backwards the amount driven forwards for cliff avoidance
        case reverse:
          // check if current encoder values have met/exceeded target values
          if (abs(l_enc.getPosition()) >= reverse_dist && abs(r_enc.getPosition()) >= reverse_dist) {
            drive_motor(0,0);
            update_global_pos();
            movingActions = cliff_turn;
          }
          else {
            PID_drive(-120, -120);
          }

          break;
        // special turn unique to dealing with cliff avoidance
        case cliff_turn:
          gyro_turn(95); // execute turn
          l_enc.resetPosition(); // zero left encoder
          r_enc.resetPosition(); // zero right encoder
          event = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // get current heading

          // update setpoints
          gyro_setpoint = event.x();
          gtarget = event.x();
          movingActions = jump;
          Stack.push(new left());
          break;
      }
      break;
    // Deals with checking for and recognizing a flame in the field of view
    case detect:
      // move servos into searching position
      ledcWrite(SERV1_CHANNEL, servo1_freq);
      ledcWrite(SERV2_CHANNEL, servo2_freq);

      // grab values from the IR camera
      IRcam.requestPosition();
      if (IRcam.available()) {
        fire_x_pos = IRcam.readY(0);
        fire_y_pos = IRcam.readX(0);

        // printResult();
        // when flame is found, start moving to extinguish
        if (fire_x_pos < 350 && fire_x_pos > 250) {
          drive_motor(0, 0); // stop movement
          update_global_pos(); // update global position
          lcd.setCursor(0,0);
          lcd.print("Flame Found"); // give visual notification that flame is found
          delay(3000);
          actions = attack;
        }
        else {
          actions = drive; // continue driving if no flame encountered
        }
      }
      break;
    // Handles going up to the flame and putting in out
    case attack:
      switch (attackingActions) {
        // turn to face the flame directly
        case faceFlame:
          servo2_freq = 6200;

          gyro_turn(90); // execute turn
          l_enc.resetPosition(); // zero left encoder
          r_enc.resetPosition(); // zero right encoder

          Stack.push(new left());

          // update setpoints
          gyro_setpoint = event.x();
          attackingActions = approach;
          break;
        // drive up to a certain distance from the flame
        case approach:
          if (frontDistanceToWall() >= 10) {
            IRcam.requestPosition();

            // update IR camera values
            if (IRcam.available()) {
              fire_x_pos = IRcam.readY(0);
              fire_y_pos = IRcam.readX(0);

              // printResult();
            }

            // turn the robot and tilt the camera up/down to stay centered on flame
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

            ledcWrite(SERV1_CHANNEL, servo1_freq);
            ledcWrite(SERV2_CHANNEL, servo2_freq);
          }
          // move on to blowing out flame once in range
          else {
            drive_motor(0, 0); // stop movement
            update_global_pos(); // update global position
            aim_timeout = millis();
            attackingActions = aim;
          }
          break;
        // makes final adjustments to blow out the flame
        case aim:
          // update IR camera values
          IRcam.requestPosition();
          if (IRcam.available()) {
            fire_x_pos = IRcam.readY(0);
            fire_y_pos = IRcam.readX(0);

            // printResult();
          }

          // move the servos to get roughly centered on the flame
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

          // move on to blow out flame after either centering or timing out
          if (fire_x_pos < 450 && fire_x_pos > 50 && fire_y_pos < 450 && fire_y_pos > 150) {
            attackingActions = extinguish;
          }
          if (millis() - aim_timeout >= 5000) {
            attackingActions = extinguish;
          }

          ledcWrite(SERV1_CHANNEL, servo1_freq);
          ledcWrite(SERV2_CHANNEL, servo2_freq);

          break;
        // Blows out the flame
        case extinguish:
          // run the fan for 20 seconds to blow out the flame
          run_fan(255);
          delay(20000);

          // display visual notification that the flame was blown out
          lcd.setCursor(0,0);
          lcd.print("Flame Blown");

          // update IR camera values
          IRcam.requestPosition();
          if (IRcam.available()) {
            fire_x_pos = IRcam.readY(0);
            fire_y_pos = IRcam.readX(0);

            // printResult();
          }
          delay(5000);

          // check that the flame was blown out
          if (fire_x_pos == 1023 && fire_y_pos == 1023) {
            run_fan(0); // turn off the fan

            event = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // get current heading

            // update global position with offset to get candle position
            global_xpos += (frontDistanceToWall() / 2.54) * cos(event.x()*(M_PI/180));
            global_ypos += (frontDistanceToWall() / 2.54) * sin(event.x()*(M_PI/180));

            actions = backtrack;
          }
          break;
      }
      break;
    // Handles returning to the original starting position
    case backtrack:
      gyro_turn(180); // execute turn
      Stack.action(); // go through the actions kept track of in reverse order
      drive_motor(0, 0); // stop movement
      actions = end;

      break;
    // Keeps the robot stopped while it's on
    case end:
      // display visual notification that program has ended
      lcd.setCursor(0, 0);
      lcd.print("End");

      drive_motor(0, 0); // stop movement
      break;
  }
}

// Prints out the output values of the IR camera
void printResult() {
  Serial.print(fire_x_pos);
  Serial.print(",");

  Serial.print(fire_y_pos);
  Serial.println(";");
}

// interrupt for the left encoder
void lenc_isr() {
  l_enc.loop();
}

// interrupt for the right encoder
void renc_isr() {
  r_enc.loop();
}
