
#include "Arduino.h"
#include "actionsAbstract.h"

class stack{
public:
  void push(actionsAbstract* obj);
  actionsAbstract* pop();
  void action();
	
private:
	actionsAbstract* arr[30];
 int len;

};
