#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <SoftwareSerial.h>
#include "Config.h"
#include "Externs.h"

class Communication {
  public:
    Communication();
    void init();
    void print();
    
  private:
    SoftwareSerial port;
};

#endif
