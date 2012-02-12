#include "SimSensors.h"

SimSensors::SimSensors(){

}

void SimSensors::init() {
  Serial.begin(SERIAL_RATE);
  navData.estLocation.latitude = 30.1804008483;
  navData.estLocation.longitude = -97.8398818969;
  navData.curWindSpeed.magnitude = 0;
  navData.curWindSpeed.direction = 0;
  navData.curAirSpeed.magnitude = 10;
  navData.curAirSpeed.direction = 90;
  sensorData.curLocation.latitude = 20.000000000;
  sensorData.curLocation.longitude = -80.00000000;
  sensorData.gpsUpdated = 0;
  sensorData.gpsBearing = 270;
  sensorData.magBearing = 90;
  sensorData.airSpeed = 10;
}

//
// update - gets latest data from sensors if it is time to read them again
//
void SimSensors::update() {
  sensorData.magBearing += 5;
  navData.curAirSpeed.direction += 5;
}

void SimSensors::mpuDataInt() {
//  mpu.dataInt();  
}
