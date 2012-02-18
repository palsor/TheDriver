#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "Externs.h"

#define BAROMETER_READ_ADDRESS 0xEF
#define BAROMETER_WRITE_ADDRESS 0xEE

#define REG_CTRL 0xF4
#define REG_EEPROM_BEGIN 0xAA
#define REG_EEPROM_END 0xBF
#define REG_DATA_MSB 0xF6
#define REG_DATA_LSB 0xF7
#define REG_DATA_XLSB 0xF8

class Barometer {
  public:
    Barometer();
    void init();
    void update();
    
  private:
    int eeprom[11];
    int state;
};

#endif
