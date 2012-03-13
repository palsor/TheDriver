#include <Servo.h>
#include <Wire.h>
#include <SPI.h>

#include "Config.h"
#include "Captain.h"
#include "Navigator.h"
#include "Pilot.h"
#include "Communication.h"
#include "Structs.h"
#include "Sensors.h"


// globals data structs
SensorData sensorData;
CaptData captData;
NavData navData;
PilotData pilotData;
ErrorData errorData;
DebugData debugData;

// working objects
Sensors sensors;
Captain captain;
Navigator navigator;
Pilot pilot;
Communication comms;

void setup() {
  
  // init SPI bus - must come before sensor and comms init
  pinMode(MPU_SS_PIN, OUTPUT);
  digitalWrite(MPU_SS_PIN, HIGH);
  pinMode(MINI_SS_PIN, OUTPUT);
  digitalWrite(MINI_SS_PIN, HIGH);
  pinMode(SPI_SLAVE_ACK_PIN, INPUT);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
  delay(10);
  
  // init our objects
  pilot.init();
  navigator.init();
  sensors.init();
  comms.init();
  
  // setup interrupts - must occur after sensor init
  attachInterrupt(0, mpuDataInt, RISING);
  
  // setup course waypoints
//  navigator.addWaypoint(30.362757,-97.90962);  // 13233 brt sky  
//  navigator.addWaypoint(30.359468,-97.904153); // Capella & Quinlan
//  navigator.addWaypoint(30.389732,-97.882315);  // quinlan & 620
//  navigator.addWaypoint(30.340962,-97.968588);  // lohman's crossing & 620
//  navigator.addWaypoint(30.4038,-97.853969);  // 4 pts
//  navigator.addWaypoint(30.455183,-97.826045);  // 620 & anderson mill
//  navigator.addWaypoint(30.447075,-97.811361);  // anderson mill @ mex food
//  navigator.addWaypoint(30.356086,-97.908718); // Quinlan & Bright Sky
//  navigator.addWaypoint(30.357274,-97.909227);  // brt sky & casseopia
//  navigator.addWaypoint(30.359007,-97.906118);  // casseopia & little dipper
//  navigator.addWaypoint(30.360123,-97.909407);  // little dipper brt sky
//  navigator.addWaypoint(30.1804008483,-97.8398818969);  // lk travis
//  navigator.addWaypoint(30.4038,-97.853969);  // 4 pts
//  navigator.addWaypoint(30.429947,-97.921314);  // c&c
//  navigator.addWaypoint(42.35111111,-71.04083333);  // nyc
//  navigator.addWaypoint(50.36388889,-4.15694444);  // london

  navigator.addWaypoint(30.359468,-97.904153); // Capella & Quinlan
  navigator.addWaypoint(30.356086,-97.908718); // Quinlan & Bright Sky
  navigator.addWaypoint(30.357274,-97.909227);  // brt sky & casseopia
  navigator.addWaypoint(30.359007,-97.906118);  // casseopia & little dipper
  navigator.addWaypoint(30.360123,-97.909407);  // little dipper brt sky

  navigator.beginNavigation();

} 

void loop() {
  sensors.update();    // read from the sensors
  captain.update();    // update state machine
  navigator.update();  // update navigation calculations
  pilot.update();      // update plane controls based on desired navigation
  comms.sendData();    // send data to arduino mini
}

void mpuDataInt() {
  sensors.mpuDataInt();  
}
