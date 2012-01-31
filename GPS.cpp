#include "GPS.h"

GPS::GPS() : gpsParser() {}
  
void GPS::init() {
  // Config serial mux for GPS
  pinMode(GPS_MUX_PIN,OUTPUT);
  digitalWrite(GPS_MUX_PIN, HIGH);
  
  // setup serial communication to GPS
  Serial.begin(SERIAL_RATE);
  Serial.flush();
  
  // configure GPS
  Serial.print(MTK_SET_RMC);
  Serial.print(MTK_OUTPUT_10HZ);
  
  if (WAIT_FOR_GPS_LOCK) {
    long lat, lon;
    unsigned long fix_age;
    gpsParser.get_position(&lat, &lon, &fix_age);
  
    while (fix_age == TinyGPS::GPS_INVALID_AGE) {
      if (Serial.available() > 0) {
        while (Serial.available() > 0) {
          if (gpsParser.encode(Serial.read())) {
            gpsParser.get_position(&lat, &lon, &fix_age);
          }
        }
      } 
    }
  } // if (WAIT_FOR_GPS_LOCK)
  
}

//
// update - read data from the GPS if available
//
void GPS::update() {
  long lat = 0;
  long lon = 0;
  unsigned long fix_age;
  
  while (Serial.available() > 0) {
    if (gpsParser.encode(Serial.read())) {
      gpsParser.get_position(&lat, &lon, &fix_age);
      sensorData.curLocation.latitude = ((float)lat)/100000;
      sensorData.curLocation.longitude = ((float)lon)/100000;
      
      sensorData.gpsBearing = (float)(gpsParser.course()) / 100; // degrees
      sensorData.gpsSpeed = (float)(gpsParser.speed()) / 100; // knot
    }
  }
}
