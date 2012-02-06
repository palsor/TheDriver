#include "MPU6000.h"

//
// constructor
//
MPU6000::MPU6000() {}

// 
// init - setup the MPU
//
void MPU6000::init() {
  
}

// 
// update - read from the MPU, do any necessary calculations, and update data structures
//
void MPU6000::update() {
  
}

//
// MPU6000 utility functions
//
byte MPU6000::spiRead(byte reg)
{
//  byte dump;
//  byte return_value;
//  byte addr = reg | 0x80; // Set most significant bit
//  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
//  dump = SPI.transfer(addr);
//  return_value = SPI.transfer(0);
//  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
//  return(return_value);
}

void MPU6000::spiWrite(byte reg, byte data)
{
//  byte dump;
//  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
//  dump = SPI.transfer(reg);
//  dump = SPI.transfer(data);
//  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
}

void MPU6000::dataInt()
{
//  newdata++;
}
