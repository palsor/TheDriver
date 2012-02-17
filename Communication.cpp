#include "Communication.h"

//
// constructor
// 
Communication::Communication() {}

//
// init
//
void Communication::init() {
  int structToTrans = 0;
}

//
// sendData - send data over SPI, protocol is [AAAA][struct_id][data][5555][checksum], only data is included in checksum
//
void Communication::sendData() {
  byte checksum = 0;
  byte* structPtr;
  byte id = structToTrans;
  int length = 0;
  
  if (structToTrans == SENSOR_DATA) {
    structPtr = (byte*)&sensorData;
    length = sizeof(SensorData);
    structToTrans++;
  }
  else if (structToTrans == NAV_DATA) {
    structPtr = (byte*)&navData;
    length = sizeof(NavData);
    structToTrans++;
  }
  else if (structToTrans == ERROR_DATA) {
    structPtr = (byte*)&errorData;
    length = sizeof(ErrorData);
    structToTrans++;
  }
  else if (structToTrans == DEBUG_DATA) {
    structPtr = (byte*)&debugData;
    length = sizeof(DebugData);
    structToTrans = 0;
  }
    
  digitalWrite(MINI_SS_PIN, LOW);
  
  transmit(0xAA);
  transmit(0xAA);
  transmit(PILOT_DATA);
  
  // send pilot data
  for (byte* ptr = (byte*)&pilotData; ptr < (byte*)&pilotData + sizeof(PilotData); ptr++) {
    transmit(*ptr);
    checksum += *ptr;
  }
  
  transmit(0x55);
  transmit(0x55);
  transmit(checksum);
  
  checksum = 0;
  
  transmit(0xAA);
  transmit(0xAA);
  transmit(id);
  
  // send whatever other struct we're sending this iteration
  for (byte* ptr = structPtr; ptr < structPtr + length; ptr++) {
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
  delayMicroseconds(SPI_TX_DELAY);
}
