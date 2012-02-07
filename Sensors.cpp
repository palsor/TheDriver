#include "Sensors.h"

Sensors::Sensors() : gps(), compass(), mpu() {}

void Sensors::init() {
  gps.init();
  compass.init();
  mpu.init();  
}

//
// update - gets latest data from sensors if it is time to read them again
//
void Sensors::update() {
    compass.update();
    gps.update();
    mpu.update();
}

void Sensors::mpuDataInt() {
  mpu.dataInt();  
}
