#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPS.h>

#include "Waypoint.h"
#include "Config.h"
#include "Externs.h"

#define MTK_SET_BINARY	"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA	"$PGCMD,16,1,1,1,1,1*6B\r\n"
#define MTK_SET_RMC     "$PGCMD,16,1,0,0,0,0*6B\r\n"

#define MTK_OUTPUT_1HZ	"$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_2HZ	"$PMTK220,500*2B\r\n"
#define MTK_OUTPUT_4HZ	"$PMTK220,250*29\r\n"
#define MTK_OTUPUT_5HZ	"$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ	"$PMTK220,100*2F\r\n"

class GPS {
  public:
    GPS();
    void init();
    float getBearing();
    float getSpeed();
    float getHorzDil();
    Waypoint* getCurLocation();
    void update();
  private:
    TinyGPS gpsParser;
    Waypoint curLocation;
    float bearing;
    float speed;
};

#endif
