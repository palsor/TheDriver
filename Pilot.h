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
    void sweepControls();  // diagnostic/test sweep of the controls
    
  private:
    unsigned long curUpdateTime;
    unsigned long lastUpdateTime;
    unsigned long dt;  // deltaTime since last loop iteration

    void updateThrottleControl();
    void updateYawControl();
    void updatePitchControl();
    void updateRollControl();

    float throttleMaintainCruiseAirSpeed();
    float rudderMaintainBearing();
    float elevatorMaintainCruiseAltitude();
    
    boolean yawSweepDir;
    boolean pitchSweepDir;
    boolean rollSweepDir;

    float fmap(float var, float min1, float max1, float min2, float max2);  // float version of the map function
    float clipMechanicalAngle(float angle, int mechMax);
    float calcMinimumAngle(float curBearing, float targBearing);  // calculates a minimum angle between bearings. The result is always between -179 and 180 degrees

};

#endif
