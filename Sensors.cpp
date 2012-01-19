#include "Sensors.h"

Sensors::Sensors() : gps(), compass() {}

void Sensors::init() {
  lastGpsTime = 0;
  lastCompassTime = 0;
  lastMpuTime = 0;
  
  gps.init();
  compass.init();
}

// 
// getter and setters
//
Waypoint* Sensors::getCurLocation() { return gps.getCurLocation(); }
float Sensors::getGpsBearing() { return gps.getBearing(); }
float Sensors::getGpsSpeed() { return gps.getSpeed(); }
float Sensors::getMagBearing() { return compass.getMagBearing(); }

//
// update - gets latest data from sensors if it is time to read them again
//
void Sensors::update() {
  unsigned long curTime = millis();
  
  //if (curTime - lastGpsTime >= GPS_RT_RATE) {
    gps.update();
    lastGpsTime = curTime;
  //}
  
  if (curTime - lastCompassTime >= COMPASS_RT_RATE) {
    compass.update();
    lastCompassTime = curTime;
  }
}
