#include "Arduino.h"
#include "Wire.h"
#include "DFRobotIRPosition.h"
#include "esp32-hal-ledc.h"

#define SERVO_1 15
#define SERVO_2 23

#define COUNT_LOW_1 1450
#define COUNT_HIGH_1 4200
#define COUNT_LOW_2 1450
#define COUNT_HIGH_2 8000

#define TIMER_WIDTH 16

int servo1_freq = 3500;
int servo2_freq = 6000;

DFRobotIRPosition IRcam;
int x_pos;
int y_pos;

void setup() {
  Serial.begin(115200);

  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(SERVO_1, 1);   // GPIO 23 assigned to channel 1

  ledcSetup(2, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(SERVO_2, 2);   // GPIO 23 assigned to channel 1

  IRcam.begin();
}

void loop() {
  IRcam.requestPosition();
  if (IRcam.available()) {
    x_pos = IRcam.readY(0);
    y_pos = IRcam.readX(0);

    printResult();
  }

  if (x_pos != 1023 && y_pos != 1023) {
    if (x_pos > 350) {
      servo2_freq += 3;
    }
    if (x_pos < 250) {
      servo2_freq -= 3;
    }

    if (y_pos > 350) {
      servo1_freq += 3;
    }
    if (y_pos < 250) {
      servo1_freq -= 3;
    }
  }

  ledcWrite(1, servo1_freq);
  ledcWrite(2, servo2_freq);

  Serial.print(servo1_freq);
  Serial.print(",");

  Serial.print(servo2_freq);
  Serial.println(";");
}

void printResult() {
  Serial.print(x_pos);
  Serial.print(",");

  Serial.print(y_pos);
  Serial.print(";");
}
