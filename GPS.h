#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

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
    void update();
  private:
    void readData();
    void parseData();
    int state;
    int totalBytes;
    int bytesRead;
    byte buff[32];
    byte ckA;
    byte ckB;
};

#endif
