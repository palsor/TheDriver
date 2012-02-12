#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#include "Config.h"
#ifdef SIMULATION_MODE
#include "SimSensors.h"
#else
#include "Sensors.h"
#endif
#include "Controller.h"
#include "Navigator.h"
#include "Pilot.h"
#include "Communication.h"
#include "Structs.h"

// globals
SensorData sensorData;
NavData navData;
PilotData pilotData;
ErrorData errorData;
DebugData debugData;

#ifdef SIMULATION_MODE
SimSensors sensors;
#else
Sensors sensors;
#endif
Controller controller;
Navigator navigator;
Pilot pilot;
Communication comms;

unsigned long nextCommTime = 0;

void setup() {
  
  // init SPI bus - must come before sensor and comms init
  pinMode(MPU_SS_PIN, OUTPUT);
  digitalWrite(MPU_SS_PIN, HIGH);
  pinMode(MINI_SS_PIN, OUTPUT);
  digitalWrite(MINI_SS_PIN, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
  delay(10);
  
  // init our objects
  controller.init();
  pilot.init();
  navigator.init();
  sensors.init();
  comms.init();
  
  // setup interrupts - must occur after sensor init
  attachInterrupt(0, mpuDataInt, RISING);
  
  // setup course waypoints
//  navigator.addWaypoint(30.362757,-97.90962);  // brt sky  
  navigator.addWaypoint(30.1804008483,-97.8398818969);  // lk travis
  navigator.addWaypoint(30.4038,-97.853969);  // 4 pts
  navigator.addWaypoint(30.429947,-97.921314);  // c&c
  navigator.addWaypoint(42.35111111,-71.04083333);  // nyc
  navigator.addWaypoint(50.36388889,-4.15694444);  // london
  
  navigator.beginNavigation();
} 

void loop() {
  sensors.update();    // read from the sensors
  navigator.update();  // update navigation calculations
  pilot.update();      // update plane controls based on desired navigation
  controller.update(); // send new signals to servos and motor
  
  comms.sendData();    // send data to arduino mini
  
  #ifdef SIMULATION_MODE
  delay(500);
  #endif
}

void mpuDataInt() {
  sensors.mpuDataInt();  
}
