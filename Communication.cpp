#include "Communication.h"

//
// constructor
// 
Communication::Communication() {}

//
// init
//
void Communication::init() {

}

//
// sendData - send data over SPI
//
void Communication::sendData() {
  byte checksum = 0;
    
  digitalWrite(MINI_SS_PIN, LOW);
  
  transmit(0xAA);
  transmit(0xAA);

  for (byte* ptr = (byte*)&sensorData; ptr < (byte*)&sensorData + sizeof(SensorData); ptr++) {
    transmit(*ptr);
    checksum += *ptr;
  }
  
  for (byte* ptr = (byte*)&navData; ptr < (byte*)&navData + sizeof(NavData); ptr++) {
    transmit(*ptr);
    checksum += *ptr;
  }
  
  for (byte* ptr = (byte*)&pilotData; ptr < (byte*)&pilotData + sizeof(PilotData); ptr++) {
    transmit(*ptr);
    checksum += *ptr;
  }
  
  for (byte* ptr = (byte*)&errorData; ptr < (byte*)&errorData + sizeof(ErrorData); ptr++) {
    transmit(*ptr);
    checksum += *ptr;
  }
  
  for (byte* ptr = (byte*)&debugData; ptr < (byte*)&debugData + sizeof(DebugData); ptr++) {
    transmit(*ptr);
    checksum += *ptr;
  }
  
  transmit(0x55);
  transmit(0x55);
  transmit(checksum);
  
  digitalWrite(MINI_SS_PIN, HIGH);
}

void Communication::transmit(byte byteToTrans) {
  byte dump = SPI.transfer(byteToTrans);
  delayMicroseconds(5);
}
