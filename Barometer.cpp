#include "Barometer.h"

//
// constructor
//
Barometer::Barometer() {}

//
// init
//
void Barometer::init() {
  
  // init variables
  state = 0;
  
  // read the calibration eerpom
  byte buff[22];
  int i = 0;
  
  Wire.beginTransmission(BAROMETER_WRITE_ADDRESS); 
  Wire.write(REG_EEPROM_BEGIN);
  Wire.endTransmission();

  Wire.requestFrom(BAROMETER_READ_ADDRESS, 22);    // request 22 bytes from device
  while((Wire.available()) && (i < 22))   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();
  
  // move the raw bytes into real values
  for (int i = 0; i < 11; i++) {
    eeprom[i] = (buff[i*2] << 8) | buff[i*2+1];
  } 
  
}

//
// update
//
void Barometer::update() {
  
}
