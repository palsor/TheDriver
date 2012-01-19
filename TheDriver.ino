#include <Servo.h>
#include <Wire.h>
#include <TinyGPS.h>
#include "Sensors.h"
#include "Controller.h"
#include "Navigator.h"
#include "Config.h"

Sensors sensors;
Controller controller;
Navigator navigator;

void setup() {
  Serial.begin(SERIAL_RATE);
  controller.init();
  sensors.init();
  
  // setup waypoints
  //navigator.addTrackWaypoint(30.362757,-97.90962);  // brt sky
  navigator.addTrackWaypoint(30.40334,-97.893248);  // lk travis
  navigator.addTrackWaypoint(30.4038,-97.853969);  // 4 pts
  navigator.addTrackWaypoint(30.429947,-97.921314);  // c&c
  navigator.addTrackWaypoint(42.35111111,-71.04083333);  // nyc
  navigator.addTrackWaypoint(50.36388889,-4.15694444);  // london
} 

void loop() {
  sensors.update();
  controller.updateSteering(sensors.getMagBearing(), sensors.getGpsBearing(), navigator.getBearingToDest(sensors.getCurLocation()), sensors.getGpsSpeed());
}
