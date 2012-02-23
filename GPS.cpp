#include "GPS.h"

GPS::GPS() {}
  
void GPS::init() {
  // Config serial mux for GPS
  pinMode(GPS_MUX_PIN,OUTPUT);
  digitalWrite(GPS_MUX_PIN, HIGH);
  
  // setup serial communication to GPS
  Serial.begin(SERIAL_RATE);
  Serial.flush();
  
  // configure GPS
  Serial.print(MTK_SET_BINARY);
  Serial.print(MTK_OUTPUT_4HZ);
  
  totalBytes = 0;
  bytesRead = 0;
  state = 0;
  ckA = 0;
  ckB = 0;
  
  sensorData.gpsUpdated = 0;
  
  if (WAIT_FOR_GPS_LOCK) {
    while (sensorData.gpsFixType != 3) {
      readData();  
    }
  }
  
}

//
// update - read data from the GPS if available
//
void GPS::update() {
  sensorData.gpsUpdated = 0;
  readData();
}

//
// readData - read available serial data and run it through state machine
//
void GPS::readData() {
  while (Serial.available() > 0) {
    byte c = Serial.read();
    
    if (state == 0) {
      if (c == 0xD0)
        state = 1;
    }
    else if (state == 1) {
      if (c == 0xDD)
        state = 2;
      else
        state = 0;
    }
    else if (state == 2) {
      if (c == 32) {
        state = 3;
        totalBytes = c;
        ckA += c;
        ckB += ckA;
      }
      else
        state = 0;
    }
    else if (state == 3) {
      buff[bytesRead] = c;
      bytesRead++;
      ckA += c;
      ckB += ckA;
      if (bytesRead == totalBytes) {
        state = 4;
        bytesRead = 0;  
      }
    }
    else if (state == 4) {
      if (c == ckA) {
        state = 5;
      } else {
        state = 0;
        ckA = 0;
        ckB = 0;
      }    
    }
    else if (state == 5) {
      if (ckB != c) {
        state = 0;
        ckA = 0;
        ckB = 0;
      } else {
        parseData();  
      }
    }
  }
}

//
// parseData - got good data, now we need to parse it
//
void GPS::parseData() {
  if (buff[21] == 3) // qualify GPS data with 3d fix types
  {
    sensorData.gpsUpdated = 1;
    sensorData.curLocation.latitude = (float)(((long)buff[3] << 24) + ((long)buff[2] << 16) + ((long)buff[1] << 8) + (long)buff[0]) / 1000000;
    sensorData.curLocation.longitude = (float)(((long)buff[7] << 24) + ((long)buff[6] << 16) + ((long)buff[5] << 8) + (long)buff[4]) / 1000000;
    sensorData.gpsAltitude = (float)(((long)buff[11] << 24) + ((long)buff[10] << 16) + ((long)buff[9] << 8) + (long)buff[8]) / 100;
    sensorData.gpsSpeed = (float)(((long)buff[15] << 24) + ((long)buff[14] << 16) + ((long)buff[13] << 8) + (long)buff[12]) / 100; // m/s
    sensorData.gpsBearing = (float)(((long)buff[19] << 24) + ((long)buff[18] << 16) + ((long)buff[17] << 8) + (long)buff[16]) / 100; 
    sensorData.gpsSatellites = buff[20];
    sensorData.gpsFixType = buff[21];
    sensorData.gpsHDOP = (float)((long)buff[29] << 8 + (long)buff[28]) / 100;
  }
}
