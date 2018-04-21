#include <PID_v1.h>

#define L_MOTOR_A 32
#define L_MOTOR_B 33
#define R_MOTOR_A 25
#define R_MOTOR_B 26

#define LA_CHANNEL 2
#define LB_CHANNEL 3
#define RA_CHANNEL 4
#define RB_CHANNEL 5

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
double setpoint, input, output;
double Kp = 2, Ki = 5, Kd = 1;


enum drivingStates {
  drive
} actions;

enum movingStates {
  forward,
  turnLeft
} movingActions;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//sets up all the pins and library functions
void setup() {
  pinMode(fronttrigPin, OUTPUT); // Sets the fronttrigPin as an Output
  pinMode(frontechoPin, INPUT); // Sets the frontechoPin as an Input
  pinMode(lefttrigPin, OUTPUT); // Sets the lefttrigPin as an Output
  pinMode(leftechoPin, INPUT); // Sets the leftechoPin as an Input
  pinMode(righttrigPin, OUTPUT); // Sets the righttrigPin as an Output
  pinMode(rightechoPin, INPUT); // Sets the rightechoPin as an Input
  Serial.begin(115200); // Starts the serial communication

  setpoint = 14.5;
  myPID.SetMode(AUTOMATIC);

  ledcSetup(LA_CHANNEL, 100, 8);
  ledcAttachPin(L_MOTOR_A, LA_CHANNEL);

  ledcSetup(LB_CHANNEL, 100, 8);
  ledcAttachPin(L_MOTOR_B, LB_CHANNEL);

  ledcSetup(RA_CHANNEL, 100, 8);
  ledcAttachPin(R_MOTOR_A, RA_CHANNEL);

  ledcSetup(RB_CHANNEL, 100, 8);
  ledcAttachPin(R_MOTOR_B, RB_CHANNEL);

  actions = drive;
  movingActions = forward;
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

void loop() {
  switch (actions) {
    case drive:
      switch (movingActions) {
        case forward:
          if (frontDistanceToWall() >= 8) {
            input = rightDistanceToWall();
            myPID.Compute();

            if (rightDistanceToWall() < 14) {
              drive_motor(100, 120 + output);
            }
            else if (rightDistanceToWall() > 15) {
              drive_motor(120 + output, 100);
            }
            else {
              drive_motor(120, 120);
            }
          }
          else {
            drive_motor(0,0);
            movingActions = turnLeft;
          }

          break;
        case turnLeft:
          break;
      }
  }
}
