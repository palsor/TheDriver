#include "Sensors.h"

Sensors::Sensors() : gps(), compass() {}

void Sensors::init() {
  gps.init();
  compass.init();  
}

//
// update - gets latest data from sensors if it is time to read them again
//
void Sensors::update() {
    compass.update();
    gps.update();
}
