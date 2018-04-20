#pragma once

#include "actionsAbstract.h"

class right :public actionsAbstract{
public:
   void action();
  void drive_motor(int lmotor, int rmotor);
	
private:
	 #define L_MOTOR_A 32
  #define L_MOTOR_B 33
  #define R_MOTOR_A 25
  #define R_MOTOR_B 26
  #define LA_CHANNEL 2
  #define LB_CHANNEL 3
  #define RA_CHANNEL 4
  #define RB_CHANNEL 5
};
