#include "SingleWire.h"
//
// constructor
//
SingleWire::SingleWire() {}

//
// init
//
void SingleWire::init() {
  pinMode(AIRSPEED_PIND, INPUT);
}

//
// calibrate - calibrate any sensors that need it
// 
void SingleWire::calibrate(int calRound) {
  // update calRound value, need 1-5 instead of 0-4;
  calRound++; 
  // calculate updated airspeedOffset value for this calibration round
  airspeedOffset = airspeedOffset * (1.0f - 1.0f/(float)calRound) + ((float)analogRead(AIRSPEED_PIN) * VREF50 / MAX_ADC_RANGE - 2.5) * 1.0f/(float)calRound;   
}

//
// readAirspeed - read the airspeed from the analog airspeed sensor
//
float SingleWire::readAirspeed() {
  // read analog pin
  float voltage = (float)analogRead(AIRSPEED_PIN) * VREF50 / MAX_ADC_RANGE - airspeedOffset; 
  // convert voltage to a dynamic pressure in n/m^2
  float pressure = ((voltage / VREF50) - .5) / .2 * 1000.0;  
  
  // calculate the airspeed from the dynamic pressure
  
  // if pressure is less than 0, the sqrt function won't work, this shouldn't happen in normal, but potentially happens on the ground
  if (pressure < 0)
    pressure = 0;
  
  return(sqrt(pressure * 2.0 / RHO));
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
