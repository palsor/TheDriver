#ifndef SIMSENSORS_H
#define SIMSENSORS_H

#include <Arduino.h>

#include "Config.h"
#include "Externs.h"

class SimSensors {
  public:
    SimSensors();
    void init();
    void update();
    void mpuDataInt();
    
   private:

};

#endif
