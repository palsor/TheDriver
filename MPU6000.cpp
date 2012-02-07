#include "MPU6000.h"

//
// constructor
//
MPU6000::MPU6000() {}

// 
// init - setup the MPU
//
void MPU6000::init() {    
    spiWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET); // Chip reset
    delay(100);
    
    spiWrite(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ); // Wake up device and select GyroZ clock (better performance)
    spiWrite(MPUREG_USER_CTRL, BIT_I2C_IF_DIS); // Disable I2C bus (recommended on datasheet)    
    spiWrite(MPUREG_SMPLRT_DIV,19);     // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
    spiWrite(MPUREG_CONFIG, BITS_DLPF_CFG_20HZ);  // FS & DLPF   FS=2000º/s, DLPF = 20Hz (low pass filter)
    spiWrite(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000º/s
    spiWrite(MPUREG_ACCEL_CONFIG,0x08);            // Accel scale 4g (4096LSB/g)
    spiWrite(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    spiWrite(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
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
  byte dump;
  byte return_value;
  byte addr = reg | 0x80; // Set most significant bit
  digitalWrite(MPU_SS_PIN, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MPU_SS_PIN, HIGH);
  return(return_value);
}

void MPU6000::spiWrite(byte reg, byte data)
{
  byte dump;
  digitalWrite(MPU_SS_PIN, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(MPU_SS_PIN, HIGH);
  delay(1);
}

void MPU6000::dataInt()
{
  newdata++;
}
