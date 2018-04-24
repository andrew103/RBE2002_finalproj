#include <ESPRotary.h>
#include "Arduino.h"
#include "actionsAbstract.h"

class forward :public actionsAbstract {
public:
   void action();
  void drive_motor(int lmotor, int rmotor);
  void lenc_isr();
  void renc_isr();
	
private:
  double distance;
  #define ENC_CPR 2248 // number of counts per revolution
#define WHEEL_CIRCUM 8.5 // circumference of the wheel in inches

#define ENC_LA 16
#define ENC_LB 17
#define ENC_RA 13
#define ENC_RB 14
  ESPRotary l_enc = ESPRotary(ENC_LA, ENC_LB, 1);
ESPRotary r_enc = ESPRotary(ENC_RA, ENC_RB, 1);



  
};
