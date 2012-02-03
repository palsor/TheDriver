#ifndef PILOT_H
#define PILOT_H

#include <Arduino.h>
#include "Config.h"
#include "Externs.h"

class Pilot {
  public:
    Pilot();
    void init();
    void update();
    
  private:
    void updateSpeedControl();
    void updateHeadingControl();
};

#endif
