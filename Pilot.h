#ifndef PILOT_H
#define PILOT_H

#include <Arduino.h>
#include "Config.h"
#include "Externs.h"

class Pilot {
  public:
    Pilot();
    void init();
    void update();  // update plane controls based on desired navigation
    
  private:
    unsigned long curUpdateTime;
    unsigned long lastUpdateTime;
    float maxThrottleRate;
    void updateSpeedControl();
    void updateHeadingControl();
    float clipMechanicalAngle(float angle, int mechMax);
    float manageThrottleRate();
};

#endif
