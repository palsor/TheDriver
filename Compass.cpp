#include "Compass.h"

//
// constructor
//
Compass::Compass() {}

//
// init - do any setup required
//
void Compass::init() {  
  if (READ_MODE) {
    Wire.beginTransmission(COMPASS_ADDRESS);
    Wire.write(MODE_REGISTER); 
    Wire.write((byte)CONTINUOUS_MODE); // Set continuous mode
    Wire.endTransmission(); //end transmission
  }
}

//
// update - read new data from the compass
//
boolean Compass::readRawValues(float* mag) {
  int i = 0;
  byte buff[6];
  boolean returnValue = false;

  Wire.beginTransmission(COMPASS_ADDRESS); 
  Wire.write(DATA_ADDRESS);
  Wire.endTransmission();

  Wire.requestFrom(COMPASS_ADDRESS, 6);    // request 6 bytes from device
  while((Wire.available()) && (i < 6))   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission

  if (i==6)  // All bytes received?
  {
    // MSB byte first, then LSB, X,Y,Z
    mag[0] = (float)((((int)buff[0]) << 8) | buff[1]) * MAG_X_SIGN;    // X axis
    mag[1] = (float)((((int)buff[4]) << 8) | buff[5]) * MAG_Y_SIGN;    // Y axis
    mag[2] = (float)((((int)buff[2]) << 8) | buff[3]) * MAG_Z_SIGN;    // Z axis
    returnValue = true;
  }
  
  return(returnValue);
}
