#pragma once

class actionsAbstract{
public:
 
  virtual void action() = 0;
  virtual void drive_motor(int lmotor, int rmotor) = 0;

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
