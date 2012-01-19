#ifndef CONTROLLER_H
#define CONTROLLER_H

#define X_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define X_MECHANICAL_MAX 25  // mechanical limits of servo travel

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

class Controller {
  public:
    Controller();
    void init();
    void updateSteering(float curBearing, float curGpsBearing, float targBearing, float gpsSpeed);
    
  private:
    void updateControlBearing(float curMagBearing, float curGpsBearing, float targBearing);  // calculate control bearing
    int calculateDeltaBearingAngle(float curBearing, float targBearing);  // calculates a delta angle
    int clipServoAngle(int angle);  // apply mechanical limits
    Servo steeringServo;
    int xTrim; // trim to get wheels/rudder straight
    int controlBearing;  // mag bearing controller is trying to hold
};

#endif
