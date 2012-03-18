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
  
  // init calibration values
  _AC1 = read16(REG_EEPROM_BEGIN + 0);
  _AC2 = read16(REG_EEPROM_BEGIN + 2);
  _AC3 = read16(REG_EEPROM_BEGIN + 4);
  _AC4 = read16(REG_EEPROM_BEGIN + 6);
  _AC5 = read16(REG_EEPROM_BEGIN + 8);
  _AC6 = read16(REG_EEPROM_BEGIN + 10);
  _B1 = read16(REG_EEPROM_BEGIN + 12);
  _B2 = read16(REG_EEPROM_BEGIN + 14);
  _MB = read16(REG_EEPROM_BEGIN + 16);
  _MC = read16(REG_EEPROM_BEGIN + 18);
  _MD = read16(REG_EEPROM_BEGIN + 20);
  
  // kick off a measurement
  write8(REG_CTRL, CMD_START_TEMP);
  lastTime = millis();
}

//
// readRawValues
//
boolean Barometer::readRawValues(float* temp, float* pressure) {
  boolean returnValue = false;

  // see if new data is ready
  unsigned long timeDelta = millis() - lastTime;
  //if (digitalRead(BAROMETER_PIN == HIGH)) {
  if (timeDelta > 5) {
    
    // determine whether the new data is temp or pressure
    if (state == 0) {
      // new value is temp, read, kick off new pressure measurement, and update state
      UT = read16(REG_DATA);
      
      write8(REG_CTRL, CMD_START_PRESSURE);
      lastTime = millis();
      state++;
    
    } else {
      // declare other variables needed for calculation
      long X1, X2, X3, B3, B5, B6, B7, UP, t, p;
      unsigned long B4;
      
      // new value is pressure, we should have both now
      UP = read16(REG_DATA);
      
      // calculate calibrated temperature and pressure
      X1 = (UT - (long)_AC6) * (long)_AC5 / 32768;
      X2 = (long)_MC * 2048 / (X1 + (long)_MD);
      B5 = X1 + X2;
      t = (B5 + 8) / 16;
      
      // calculate calibrate pressure
      B6 = B5 - 4000;
      X1 = ((long)_B2 * (B6 * B6 / 4096)) / 2048;
      X2 = (long)_AC2 * B6 / 2048;
      X3 = X1 + X2;
      B3 = (((long)_AC1 * 4 + X3) + 2) / 4;
      X1 = (long)_AC3 * B6 / 8192;
      X2 = ((long)_B1 * (B6 * B6 / 4096)) / 65536;
      X3 = ((X1 + X2) + 2) / 4;
      B4 = (unsigned long)_AC4 * (unsigned long)(X3 + 32768) / 32768;
      B7 = ((unsigned long)UP - B3) * 50000ul;
      if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
      } else {
        p = (B7 / B4) * 2;
      }
      X1 = (p / 256) * (p / 256);
      X1 = (X1 * 3038) / 65536;
      X2 = (-7357 * p) / 65536;
      p = p + (X1 + X2 + 3791) / 16;
      
      // update values used elsewhere
      *temp = (float)t / 10.0;
      *pressure = (float)p / 1000.0;
      
      // kick off a new temp read
      write8(REG_CTRL, CMD_START_TEMP);
      lastTime = millis();
      
      // update state machine and return value
      state = 0;
      returnValue = true;
    } 
  }

  return returnValue;  
}

//
// read16 - reads 16 bits from the barometer
//
unsigned int Barometer::read16(byte address) {
  Wire.beginTransmission(BAROMETER_ADDRESS); 
  Wire.write(address);
  Wire.endTransmission();

  Wire.beginTransmission(BAROMETER_ADDRESS);
  Wire.requestFrom(BAROMETER_ADDRESS, 2);    // request 22 bytes from device
  unsigned int MSB = Wire.read();  // receive one byte
  unsigned int LSB = Wire.read();
  Wire.endTransmission();
  
  return ((MSB << 8) | LSB);
}

//
// write - writes 8 bits to the barometer
//
void Barometer::write8(byte address, byte data) {
  Wire.beginTransmission(BAROMETER_ADDRESS); 
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}


