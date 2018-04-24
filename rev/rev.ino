/* This is the RBE 1001 Template as of
 *
 * 3/28/17
 *
 * This Template
 * is designed to run the autonomous and teleop sections of the final
 * competition. Write and test your autonomous and teleop code on your
 * own and place the code in auto.cpp or teleop.cpp respectively.
 * The functions will be called by the competition framework based on the
 * time and start button. DO NOT change this file, your code will be called
 * by the framework. The framework will pass your code a reference to the DFW
 * object as well as the amount of MS remaining.
 */

#include"stack.h"
#include"forward.h"
#include"left.h"

stack Stack ;

void setup() {
	Serial.begin(115200); // Serial output begin. Only needed for debug
	Stack.initializeStack();
}
void loop() {
  Stack.push(new forward());
  Stack.push(new left());
  Stack.action();
}
