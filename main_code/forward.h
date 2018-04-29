#include <ESPRotary.h>
#include "Arduino.h"
#include "actionsAbstract.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"
#include <ESPRotary.h>

class forward :public actionsAbstract {
public:
   forward(int dist);
   //forward();
   void action();
  void drive_motor(int lmotor, int rmotor);
  void lenc_isr();
  void renc_isr();
  double frontDistanceToWall();
  double rightDistanceToWall();
  void gyroFollow(float targetAngle);
	
private:
  double distance;
  double duration;
  ESPRotary l_enc = ESPRotary(ENC_LA, ENC_LB, 1);
ESPRotary r_enc = ESPRotary(ENC_RA, ENC_RB, 1);
const int fronttrigPin = 5;
const int frontechoPin = 36;
const int righttrigPin = 19;
const int rightechoPin = 34;
Adafruit_BNO055 bno = Adafruit_BNO055();
double Kp = 1;
int wall_setpoint = 8;
float gtarget = 0;

float travelDistance;



  
};
