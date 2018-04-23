#include "Arduino.h"
#include "Wire.h"
#include "DFRobotIRPosition.h"
#include "esp32-hal-ledc.h"

#define COUNT_LOW 1450
#define COUNT_HIGH 4200

#define TIMER_WIDTH 16

DFRobotIRPosition IRcam;
int x_pos;
int y_pos;

void setup() {
  Serial.begin(115200);

  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(23, 1);   // GPIO 22 assigned to channel 1

  myDFRobotIRPosition.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
