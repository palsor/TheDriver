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
    void transmit(byte byteToTrans);
};

#endif
