#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "Sensors.h"
#include "Controller.h"
#include "Navigator.h"
#include "Pilot.h"
#include "Communication.h"
#include "Config.h"
#include "Structs.h"

// globals
SensorData sensorData;
NavData navData;
PilotData pilotData;
ErrorData errorData;
DebugData debugData;

Sensors sensors;
Controller controller;
Navigator navigator;
Pilot pilot;
Communication comms;

unsigned long nextCommTime = 0;

void setup() {
  
  // init our objects
  controller.init();
  pilot.init();
  navigator.init();
  sensors.init();
  comms.init();
Serial.begin(SERIAL_RATE);

  
  //required initialization of course[0] to home location
//  sensors.update();
//  navigator.addWaypoint(sensorData.curLocation.latitude,sensorData.curLocation.longitude);
navigator.addWaypoint(30.362757,-97.90962);  // brt sky  

  // setup course waypoints
  navigator.addWaypoint(30.1804008483,-97.8398818969);  // lk travis
  navigator.addWaypoint(30.4038,-97.853969);  // 4 pts
  navigator.addWaypoint(30.429947,-97.921314);  // c&c
  navigator.addWaypoint(42.35111111,-71.04083333);  // nyc
  navigator.addWaypoint(50.36388889,-4.15694444);  // london
  
  navigator.beginNavigation();
} 

void loop() {
//  sensors.update(); // read from the sensors
  navigator.update(); // update navigation calculations
  pilot.update();  // update plane controls based on desired navigation
  controller.update(); // send new signals to servos and motor
  
  unsigned long curTime = millis();  
  if (curTime > nextCommTime) {
    comms.print();
    nextCommTime = curTime + COMM_OUTPUT_RATE;
  }
}
