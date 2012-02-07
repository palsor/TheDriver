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
// print - send data over serial
//
void Communication::sendData() {
  
}

//
// utility functions
//
byte Communication::spiRead(byte reg)
{
  byte dump;
  byte return_value;
  byte addr = reg | 0x80; // Set most significant bit
  digitalWrite(MINI_SS_PIN, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MINI_SS_PIN, HIGH);
  return(return_value);
}

void Communication::spiWrite(byte reg, byte data)
{
  byte dump;
  digitalWrite(MINI_SS_PIN, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(MINI_SS_PIN, HIGH);
  delay(1);
}
