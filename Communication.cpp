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
    
  digitalWrite(MINI_SS_PIN, LOW);
  
  transmitStruct(PILOT_DATA, (byte*)&pilotData, sizeof(PilotData));
  
  if (structToTrans == SENSOR_DATA) {
    transmitStruct(SENSOR_DATA, (byte*)&sensorData, sizeof(SensorData));
    structToTrans++;
  }
  else if (structToTrans == NAV_DATA) {
    transmitStruct(NAV_DATA, (byte*)&navData, sizeof(NavData));
    structToTrans = 0;  
  }
  
  digitalWrite(MINI_SS_PIN, HIGH);
}

void Communication::transmitStruct(byte id, byte* ptr, int length) {
  byte checksum = 0;
  
  transmit(0xAA);
  transmit(0xAA);
  transmit(id);
  
  for (byte* temp = ptr; temp < ptr + length; temp++) {
    transmit(*temp);
    checksum += *temp;
  }
  
  transmit(0x55);
  transmit(0x55);
  transmit(checksum);
}

void Communication::transmit(byte byteToTrans) {
  int val = digitalRead(SPI_SLAVE_ACK_PIN);
  byte dump = SPI.transfer(byteToTrans);
  while(digitalRead(SPI_SLAVE_ACK_PIN) == val) {}  
}
