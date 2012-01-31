#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <TinyGPS.h>

#include "Compass.h"
#include "GPS.h"
#include "Config.h"
#include "Externs.h"

class Sensors {
  public:
    Sensors();
    void init();
    void update();
    
   private:
     Compass compass;
     GPS gps;
};

#endif
