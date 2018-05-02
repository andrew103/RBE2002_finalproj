#pragma once

#define L_MOTOR_A 32
#define L_MOTOR_B 33
#define R_MOTOR_A 25
#define R_MOTOR_B 26
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

class actionsAbstract {
  public:
    virtual void action() = 0;
    virtual void drive_motor(int lmotor, int rmotor) = 0;
};
