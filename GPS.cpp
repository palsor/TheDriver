#include "GPS.h"

GPS::GPS() : gpsParser(), curLocation() {}
  
void GPS::init() {
  // Config serial mux for GPS
  pinMode(GPS_MUX_PIN,OUTPUT);
  digitalWrite(GPS_MUX_PIN, HIGH);
  
  Serial.flush();
  Serial.print(MTK_SET_RMC);
  Serial.print(MTK_OUTPUT_10HZ);
  
  if (WAIT_FOR_GPS_LOCK) {
    // wait for GPS to lock
    softSerial.println("Waiting for GPS to initialize...");
  
    long lat, lon;
    unsigned long fix_age;
    gpsParser.get_position(&lat, &lon, &fix_age);
  
    while (fix_age == TinyGPS::GPS_INVALID_AGE) {
      if (Serial.available() > 0) {
        while (Serial.available() > 0) {
          if (gpsParser.encode(Serial.read())) {
            gpsParser.get_position(&lat, &lon, &fix_age);
            bearing = (float)(gpsParser.course()) / 100; // degrees
            speed = (float)(gpsParser.speed()) / 100; // knot
          }
        } // while
      } // if
    }// while
  
    softSerial.println("GPS locked...");
  }
}

//
// getters and setters
//
float GPS::getBearing() { return bearing; }
float GPS::getSpeed() { return speed; }
Waypoint* GPS::getCurLocation() { return &curLocation; }

//
// update - read data from the GPS if available
//
void GPS::update() {
  long lat = 0;
  long lon = 0;
  unsigned long fix_age;
  
  while (Serial.available() > 0) {
    byte test = Serial.read();
    Serial.print((char)test);
    if (gpsParser.encode(test)) {
      gpsParser.get_position(&lat, &lon, &fix_age);
      curLocation.setLocation(((float)lat)/100000, ((float)lon)/100000);
      
      bearing = (float)(gpsParser.course()) / 100; // degrees
      speed = (float)(gpsParser.speed()) / 100; // knot
      
      #if (GPS_DEBUG)
        softSerial.print("gpsCourse: ");
        softSerial.print(bearing);
        softSerial.print("   gpsSpeed: ");
        softSerial.print(speed);
        softSerial.print("   ");
        curLocation.print();
      #endif
    }
  }
}
