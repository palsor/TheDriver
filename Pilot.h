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
    unsigned long dt;  // deltaTime since last loop iteration
    void updateSpeedControl();
    void updateHeadingControl();
    float fmap(float var, float min1, float max1, float min2, float max2);  // float version of the map function
    float clipMechanicalAngle(float angle, int mechMax);
    float maintainCruiseAirSpeed();
    float maintainCruiseAltitude();
};

#endif
