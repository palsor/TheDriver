#include "SingleWire.h"
//
// constructor
//
SingleWire::SingleWire() {}

//
// init
//
void SingleWire::init() {
  airspeedOffset = analogRead(AIRSPEED_PIN) * VREF50 / MAX_ADC_RANGE - 2.5;  
}

//
// calibrate - calibrate any sensors that need it
// 
void SingleWire::calibrate(int calRound) {
  // update calRound value, need 1-5 instead of 0-4;
  calRound++; 
  
  // calculate updated airspeedOffset value for this calibration round
  airspeedOffset = airspeedOffset * (1.0f - 1.0f/calRound) + (analogRead(AIRSPEED_PIN) * VREF50 / MAX_ADC_RANGE - 2.5) * 1.0f/calRound;   
}

//
// readAirspeed - read the airspeed from the analog airspeed sensor
//
float SingleWire::readAirspeed() {
  // read analog pin
  float voltage = analogRead(AIRSPEED_PIN) * VREF50 / MAX_ADC_RANGE - airspeedOffset;   
  // convert voltage to a dynamic pressure in n/m^2
  float pressure = ((voltage / VREF50) - .5) / .2 * 1000;  
  // calculate the airspeed from the dynamic pressure
  return(sqrt(pressure * 2 / RHO));  
}

//
// readBattery - read the battery voltage
//
float SingleWire::readBattery() {
  return (analogRead(BATTERY_PIN) * VREF50 / MAX_ADC_RANGE); 
}

// 
// readFailsafeMux - read the failsafe mux and determine what is in control
//
bool SingleWire::readFailsafeMux() {
  return false;
}
