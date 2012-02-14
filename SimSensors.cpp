#include "SimSensors.h"

SimSensors::SimSensors(){

}

void SimSensors::init() {
  Serial.begin(SERIAL_RATE);
  navData.estLocation.latitude = 30.362757;
  navData.estLocation.longitude = -97.90962;
  navData.curWindSpeed.magnitude = 0;
  navData.curWindSpeed.direction = 0;
  navData.curAirSpeed.magnitude = 10;
  navData.curAirSpeed.direction = 90;
  sensorData.curLocation.latitude = 30.362757;
  sensorData.curLocation.longitude = -97.90962;
  sensorData.gpsUpdated = 1;
  sensorData.gpsBearing = 161.7224731445;
  sensorData.magBearing = 161.7224731445;
  sensorData.airSpeed = 10;
}

//
// update - gets latest data from sensors if it is time to read them again
//
void SimSensors::update() {
  float radBearing = sensorData.magBearing * 3.14159265358979323846 / 180;
  sensorData.curLocation.latitude += (sensorData.airSpeed * 500 / 1000 / 3600) * cos(radBearing);
  sensorData.curLocation.longitude += (sensorData.airSpeed * 500 / 1000 / 3600) * sin(radBearing);
}

void SimSensors::mpuDataInt() {
//  mpu.dataInt();  
}
