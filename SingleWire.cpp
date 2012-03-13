#include "SingleWire.h"
//
// constructor
//
SingleWire::SingleWire() {}

//
// init
//
void SingleWire::init() {
  airspeedOffset = analogRead(AIRSPEED_PIN) * VREF / MAX_ADC_RANGE - 2.5;  
}

//
// readAirspeed - read the airspeed from the analog airspeed sensor
//
float SingleWire::readAirspeed() {
  // read analog pin
  float voltage = analogRead(AIRSPEED_PIN) * VREF / MAX_ADC_RANGE - airspeedOffset;   
  // convert voltage to a dynamic pressure in n/m^2
  float pressure = ((voltage / VREF) - .5) / .2 * 1000;  
  // calculate the airspeed from the dynamic pressure
  float airspeed = sqrt(pressure * 2 / RHO);

  return airspeed;  
}

//
// readBattery - read the battery voltage
//
float SingleWire::readBattery() {
  float voltage = analogRead(BATTERY_PIN) * VREF / MAX_ADC_RANGE; 
  return voltage;
}

// 
// readFailsafeMux - read the failsafe mux and determine what is in control
//
bool SingleWire::readFailsafeMux() {
  return false;
}
