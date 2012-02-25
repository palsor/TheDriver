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
    void transmitStruct(byte id, byte* ptr, int length, boolean delayAfterFirst);
    int structToTrans;
};

#endif
