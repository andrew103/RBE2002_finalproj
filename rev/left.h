
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

  float target = -1.0;
int turn_amount;
bool is_turning = false;
Adafruit_BNO055 bno = Adafruit_BNO055();
};
