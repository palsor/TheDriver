#ifndef PILOT_H
#define PILOT_H

#include <Arduino.h>
#include "Config.h"
#include "Externs.h"
#include "Constants.h"

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
    
    float speedRange;
    float throttleRange;

    void updateThrottleControl();
    void updateRudderControl();
    void updateElevatorControl();
    void updateAileronControl();

    float throttleMaintainCruiseAirSpeed();
    float rudderMaintainBearing();
    float elevatorMaintainCruiseAltitude();
    
    float vasAltAccum;  // vertical airspeed altitude accumulator for adaptive pitch integrator
    float startPalt;  // start pressure altitude adaptive pitch
    unsigned long dtAccum;  // ms since the integrator was last reset/started

    void resetVasIntegrator();
    void integrateVas();
    void calibratePitch();
    
    boolean rudderSweepDir;
    boolean elevatorSweepDir;
    boolean aileronSweepDir;

    float zeroLimitCos(float angle, float range);  // limits cos to avoid div 0s
    float fmap(float var, float min1, float max1, float min2, float max2);  // float version of the map function
    float clipMechanicalAngle(float angle, int mechMax);
    float calcMinimumAngle(float curBearing, float targBearing);  // calculates a minimum angle between bearings. The result is always between -179 and 180 degrees

};

#endif
