#ifndef CAPTAIN_H
#define CAPTAIN_H

#include <Arduino.h>

#include "Config.h"
#include "Externs.h"

class Captain {
  public:
    Captain();
    void init();
    void update();
    
  private:
    void updateState();  // navigation state machine
      boolean priorityStateChecks();  // checks for error conditions to make priority state transitions independent of current state
      void transitionState(int newState);  // transitions state and assocaited variables

};

#endif
