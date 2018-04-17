// defines pins numbers
const int fronttrigPin = 2;
const int frontechoPin = 4;
const int lefttrigPin = 16;
const int leftechoPin = 17;
const int righttrigPin = 21;
const int rightechoPin = 22;

// defines variables
long duration;
int distance;

enum drivingStates {
  drive
};

enum movingStates {
  forward, left, right
};

drivingStates actions = drive;
movingStates movingActions = forward;


//sets up all the pins and library functions
void setup() {
  pinMode(fronttrigPin, OUTPUT); // Sets the fronttrigPin as an Output
  pinMode(frontechoPin, INPUT); // Sets the frontechoPin as an Input
  pinMode(lefttrigPin, OUTPUT); // Sets the lefttrigPin as an Output
  pinMode(leftechoPin, INPUT); // Sets the leftechoPin as an Input
  pinMode(righttrigPin, OUTPUT); // Sets the righttrigPin as an Output
  pinMode(rightechoPin, INPUT); // Sets the rightechoPin as an Input
  Serial.begin(115200); // Starts the serial communication
}

int frontDistanceToWall() {
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
  return distance;
}

int leftDistanceToWall() {
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
  return distance;
}

int rightDistanceToWall() {
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
  return distance;
}

void loop() {
  switch (actions) {
    case drive:
      switch (movingActions) {
        case forward: if (frontDistanceToWall() >= 14) {
            /*
               put code for moving forward
            */
          }
        case left:  if (rightDistanceToWall() <= 14) {
            /*
               put code to turn left
            */
          }
        case right:  if (rightDistanceToWall() > 14) {
            /*
               put code to turn right
            */
          }
      }
  }
}

