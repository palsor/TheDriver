#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

#include "Compass.h"
#include "MPU6000.h"
#include "GPS.h"
#include "Config.h"
#include "Externs.h"
#include "Barometer.h"
#include "Constants.h"
#include "SingleWire.h"

class Sensors {
  public:
    Sensors();
    void init();
    void update();
    void mpuDataInt();
    
   private:
     Compass compass;
     GPS gps;
     MPU6000 mpu;
     Barometer barometer;
     SingleWire singleWire;
     
     float rotation[3][3];
     unsigned long lastUpdateTime;
     
     void updateRotationMatrix();
     void updateAirspeed();
     
     float matrixDot(float* a, float* b);
     void matrixRotate(float* bodyVec, float* earthVec);
     void matrixUnit(float* b);
     void matrixCross(float* a, float* b, float* c);
};

#endif
