#include "Pilot.h"

Pilot::Pilot() {
 
}

void Pilot::init() {
  
}

//
// update - calculate new nav data
//
void Pilot::update() {
  updateSpeedControl();
  updateHeadingControl();
}

void Pilot::updateSpeedControl() {
  
}

void Pilot::updateHeadingControl() {
  pilotData.yawValue = map(round(navData.deltaBearing),YAW_CENTER_ANGLE-YAW_MECHANICAL_MAX,YAW_CENTER_ANGLE+YAW_MECHANICAL_MAX,0,99);
}
