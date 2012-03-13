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
  lastByte = 0x00;
  debugData.spiXmtCount = 0;
  debugData.spiXmtErrorCount = 0;
  
}

//
// sendData - send data over SPI, protocol is [AAAA][struct_id][data][5555][checksum], only data is included in checksum
//
void Communication::sendData() {
    
  digitalWrite(MINI_SS_PIN, LOW);
  
  transmitStruct(PILOT_DATA, (byte*)&pilotData, sizeof(PilotData), true);
  
  if (structToTrans == SENSOR_DATA) {
    transmitStruct(SENSOR_DATA, (byte*)&sensorData, sizeof(SensorData), false);
    structToTrans++;
  }
  else if (structToTrans == NAV_DATA) {
    transmitStruct(NAV_DATA, (byte*)&navData, sizeof(NavData), false);
    structToTrans = 0;  
  }
  
  digitalWrite(MINI_SS_PIN, HIGH);
}

void Communication::transmitStruct(byte id, byte* ptr, int length, boolean delayAfterFirst) {
  byte checksum = 0;
  byte byteErrors = 0;
  
  transmit(0xAA);
  transmit(0xAA);
  transmit(id);
  
  for (byte* temp = ptr; temp < ptr + length; temp++) {
    if(transmit(*temp)) { byteErrors++; };
    checksum += *temp;
  }
  
  transmit(0x55);
  transmit(0x55);
  transmit(checksum);
  
  if(byteErrors > 0) { debugData.spiXmtErrorCount++; }
  debugData.spiXmtCount++;
}

boolean Communication::transmit(byte byteToTrans) {
  int val = digitalRead(SPI_SLAVE_ACK_PIN);
  byte slaveByte = SPI.transfer(byteToTrans);
  byte xorByteToTrans = byteToTrans ^ 0xFF;
  boolean fail = slaveByte != lastByte;
  lastByte = xorByteToTrans;
  while(digitalRead(SPI_SLAVE_ACK_PIN) == val) {}
  return(fail);
}
