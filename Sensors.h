#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <TinyGPS.h>

#include "Waypoint.h"
#include "Compass.h"
#include "GPS.h"
#include "Config.h"

class Sensors {
  public:
    Sensors();
    void init();
    Waypoint* getCurLocation();
    float getGpsBearing();
    float getGpsSpeed();
    float getMagBearing();
    void update();
    
   private:
     Compass compass;
     GPS gps;
     unsigned long lastGpsTime, lastCompassTime, lastMpuTime;
};

#endif
