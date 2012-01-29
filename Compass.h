#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Externs.h"

// Mag settings
#define READ_MODE 0x01 // 1 = continuous mode, 0 = normal mode

// Mag addresses
#define COMPASS_ADDRESS 0x1E
#define MODE_REGISTER 0x02
#define DATA_ADDRESS 0x03

// Mag register values
#define CONTINUOUS_MODE 0

class Compass {
  public:
    Compass();
    void init();
    float getMagBearing();
    void update();
  private:
    float magBearing;
};

#endif
