#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <SPI.h>

#include "Config.h"
#include "Externs.h"

class Communication {
  public:
    Communication();
    void init();
    void sendData();
    
  private:
    byte spiRead(byte reg);
    void spiWrite(byte reg, byte data);
};

#endif
