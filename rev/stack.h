
#include "Arduino.h"
#include "actionsAbstract.h"

class stack{
public:
  void push(actionsAbstract obj);
  void pop();
  void action();
	
private:
	actionsAbstract arr[20];
};
