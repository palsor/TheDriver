#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "Sensors.h"
#include "Controller.h"
#include "Navigator.h"
#include "Communication.h"
#include "Config.h"
#include "Structs.h"

// globals
SensorData sensorData;
NavData navData;
ErrorData errorData;
DebugData debugData;

Sensors sensors;
Controller controller;
Navigator navigator;
Communication comms;

unsigned long nextCommTime = 0;

void setup() {
  
  // init our objects
  controller.init();
  sensors.init();
  comms.init();
  
  // setup waypoints
  //navigator.addTrackWaypoint(30.362757,-97.90962);  // brt sky
  navigator.addTrackWaypoint(30.1804008483,-97.8398818969);  // lk travis
  navigator.addTrackWaypoint(30.4038,-97.853969);  // 4 pts
  navigator.addTrackWaypoint(30.429947,-97.921314);  // c&c
  navigator.addTrackWaypoint(42.35111111,-71.04083333);  // nyc
  navigator.addTrackWaypoint(50.36388889,-4.15694444);  // london
} 

void loop() {
  sensors.update(); // read from the sensors
  navigator.update(); // update navigation calculations
  controller.update(); // send new signals to servos and motor
  
  unsigned long curTime = millis();  
  if (curTime > nextCommTime) {
    comms.print();
    nextCommTime = curTime + COMM_OUTPUT_RATE;
  }
}
