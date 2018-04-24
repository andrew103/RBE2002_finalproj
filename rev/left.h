
#include "Arduino.h"
#include "actionsAbstract.h"
#include <Wire.h>
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"

class left :public actionsAbstract{
public:
   void action();
  void drive_motor(int lmotor, int rmotor);
	void create_target(float current);
 void gyro_turn(int amount) ;
private:
	 #define L_MOTOR_A 32
  #define L_MOTOR_B 33
  #define R_MOTOR_A 25
  #define R_MOTOR_B 26
  #define LA_CHANNEL 2
  #define LB_CHANNEL 3
  #define RA_CHANNEL 4
  #define RB_CHANNEL 5

  float target = -1.0;
int turn_amount;
bool is_turning = false;
Adafruit_BNO055 bno = Adafruit_BNO055();
};
