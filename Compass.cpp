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
void Compass::update() {
  int i = 0;
  byte buff[6];
  float magRaw[3];

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
    magRaw[0] = (float)((((int)buff[0]) << 8) | buff[1]);    // X axis
    magRaw[1] = (float)((((int)buff[4]) << 8) | buff[5]);    // Y axis
    magRaw[2] = (float)((((int)buff[2]) << 8) | buff[3]);    // Z axis
    
  }
  else {
    errorData.compassReadError = true;
  }

  // calculate heading
  float tempBearing = atan2(magRaw[1], magRaw[0]);
  
  // make heading always between 0 and 2PI
  if (tempBearing < 0)
    tempBearing += 2 * 3.14159;
  
  // convert heading to degrees and apply magnetic declination  
  tempBearing = (tempBearing * 180 / 3.14159) + MAG_DECLINATION;
  
  if (tempBearing < 0)
    tempBearing += 360;
  
  // copy to data structure
  sensorData.magBearing = tempBearing;
}
