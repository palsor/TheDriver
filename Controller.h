#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "Externs.h"

class Controller {
  public:
    Controller();
    void init();
    void update();
    
  private:
    Servo throttleServo;
    Servo pitchServo;
    Servo yawServo;
    Servo rollServo;
    void applyThrottleValue();
    void applyPitchValue();
    void applyYawValue();
    void applyRollValue();
};

#endif
