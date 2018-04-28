#include "stack.h"

void stack::push(actionsAbstract* obj){
  if ((len+1) >= 30){
    len++;
    arr[len] = obj;
    
  }
  else{
    /*
     * print on lcd about overload
     */
  }
}

actionsAbstract* stack::pop(){
 if (len >= 0){
    actionsAbstract* obj = arr[len];
    arr[len] = NULL;
    len--;
    return obj;
 }
}


void stack ::action(){
  int stackLength = len;
  for(;stackLength>=0;stackLength--){
    actionsAbstract* obj = pop();
    obj->action();
  }
}


void stack:: initializeStack(){
  len = -1;
}

