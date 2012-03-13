#ifndef SINGLEWIRE_H
#define SINGLEWIRE_H

#include <Arduino.h>

#include "Config.h"
#include "Externs.h"
#include "Constants.h"

class SingleWire {
  public:
    SingleWire();
    void init();
    float readAirspeed();
    float readBattery();
    bool readFailsafeMux();
  
  private:
    float airspeedOffset;
  
};

#endif
