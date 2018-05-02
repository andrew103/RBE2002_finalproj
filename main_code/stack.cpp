#include "stack.h"

// pushes a new execution object onto the end of the list
void stack::push(actionsAbstract* obj){
  if ((len+1) < 30){
    len++;
    arr[len] = obj;
  }
  else {
    // print on lcd about overload
  }
}

// pops off and executes the object on the end of the list
actionsAbstract* stack::pop(){
 if (len >= 0){
    actionsAbstract* obj = arr[len];
    arr[len] = NULL;
    len--;
    return obj;
 }
}

// goes through the stored list and executes each objects action
void stack ::action(){
  int stackLength = len;
  for(;stackLength>=0;stackLength--){
    actionsAbstract* obj = pop();
    obj->action();
  }
}

// initializes the array with an initial length of -1
void stack:: initializeStack(){
  len = -1;
}
