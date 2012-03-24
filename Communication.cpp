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
  
  transmitStruct(PILOT_DATA, (byte*)&pilotData, sizeof(PilotData));
  
  if (structToTrans == SENSOR_DATA) {
    transmitStruct(SENSOR_DATA, (byte*)&sensorData, sizeof(SensorData));
    structToTrans = NAV_DATA;
  }
  else if (structToTrans == NAV_DATA) {
    transmitStruct(NAV_DATA, (byte*)&navData, sizeof(NavData));
    structToTrans = CAPT_DATA;  
  }
  else if (structToTrans == CAPT_DATA) {
    transmitStruct(CAPT_DATA, (byte*)&captData, sizeof(CaptData));
    structToTrans = SENSOR_DATA;  
  }
  
  digitalWrite(MINI_SS_PIN, HIGH);
}

void Communication::transmitStruct(byte id, byte* ptr, int length) {
  byte checksum = 0;
  int byteErrors = 0;
  
  if(transmit(0xAA)) { byteErrors++; };
  if(transmit(0xAA)) { byteErrors++; };
  if(transmit(id)) { byteErrors++; };
  
  for (byte* temp = ptr; temp < ptr + length; temp++) {
    if(transmit(*temp)) { byteErrors++; };
    checksum += *temp;
  }
  
  if(transmit(0x55)) { byteErrors++; };
  if(transmit(0x55)) { byteErrors++; };
  if(transmit(checksum)) { byteErrors++; };
  
  if(byteErrors > 0) { debugData.spiXmtErrorCount++; }
  debugData.spiXmtCount++;
}

boolean Communication::transmit(byte byteToTrans) {
  int val = digitalRead(SPI_SLAVE_ACK_PIN);
  byte slaveByte = SPI.transfer(byteToTrans);
  byte xorByteToTrans = byteToTrans ^ 0xFF;
#ifdef WAIT_FOR_SLAVE_ACK
  boolean fail = slaveByte != lastByte;
  lastByte = xorByteToTrans;
  while(digitalRead(SPI_SLAVE_ACK_PIN) == val) {}
  return(fail);
#else
  return(false);
#endif
}


