#include <ESPRotary.h>
#include "Arduino.h"
#include "actionsAbstract.h"
#include <LiquidCrystal_I2C.h>
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"
#include <ESPRotary.h>

class forward :public actionsAbstract {
public:
   forward(long dist, float angle);
   //forward();
   void action();
  void drive_motor(int lmotor, int rmotor);
  static void lenc_isr();
  static void renc_isr();
  double frontDistanceToWall();
  double rightDistanceToWall();
  double leftDistanceToWall();
  void gyroFollow(float targetAngle);

private:
  static bool lenc_trigger;
  static bool renc_trigger;
  double distance;
  double duration;
  ESPRotary l_enc = ESPRotary(ENC_LA, ENC_LB, 1);
  ESPRotary r_enc = ESPRotary(ENC_RA, ENC_RB, 1);
  const int fronttrigPin = 5;
  const int frontechoPin = 36;
  const int lefttrigPin = 18;
  const int leftechoPin = 39;
  const int righttrigPin = 19;
  const int rightechoPin = 34;
  Adafruit_BNO055 bno = Adafruit_BNO055();
  LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
  double Kp = 1;
  int wall_setpoint = 8;
  float gtarget = 0;
  long travelDistance;

  int renc1 = 0;
  int renc2 = 0;
  int lenc1 = 0;
  int lenc2 = 0;
};
