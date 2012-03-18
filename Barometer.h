#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "Externs.h"

#define BAROMETER_ADDRESS 0x77

#define REG_CTRL 0xF4
#define REG_EEPROM_BEGIN 0xAA
#define REG_DATA 0xF6

#define CMD_START_TEMP 0x2E
#define CMD_START_PRESSURE 0x34

class Barometer {
  public:
    Barometer();
    void init();
    boolean readRawValues(float* temp, float* pressure);
    
  private:
    int _AC1, _AC2, _AC3, _B1, _B2, _MB, _MC, _MD;
    unsigned int _AC4, _AC5, _AC6;
    long UT;
    int state;
    unsigned long lastTime;
    
    unsigned int read16(byte address);
    void write8(byte address, byte data);
};

#endif
