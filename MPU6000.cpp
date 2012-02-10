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
  readRawValues(); 
}

//
// readRawValues - reads values straight from the gyro/accel/temp
//
void MPU6000::readRawValues() {
  int byte_H;
  int byte_L;
  
  // Read AccelX
  byte_H = spiRead(MPUREG_ACCEL_XOUT_H);
  byte_L = spiRead(MPUREG_ACCEL_XOUT_L);
  accelX = (float)((byte_H<<8)| byte_L);
  // Read AccelY
  byte_H = spiRead(MPUREG_ACCEL_YOUT_H);
  byte_L = spiRead(MPUREG_ACCEL_YOUT_L);
  accelY = (float)((byte_H<<8)| byte_L);
  // Read AccelZ
  byte_H = spiRead(MPUREG_ACCEL_ZOUT_H);
  byte_L = spiRead(MPUREG_ACCEL_ZOUT_L);
  accelZ = (float)((byte_H<<8)| byte_L);
    
  // Read Temp
  byte_H = spiRead(MPUREG_TEMP_OUT_H);
  byte_L = spiRead(MPUREG_TEMP_OUT_L);
  tempRaw = (byte_H<<8)| byte_L; 
     
  // Read GyroX
  byte_H = spiRead(MPUREG_GYRO_XOUT_H);
  byte_L = spiRead(MPUREG_GYRO_XOUT_L);
  gyroX = (float)((byte_H<<8)| byte_L);
  // Read GyroY
  byte_H = spiRead(MPUREG_GYRO_YOUT_H);
  byte_L = spiRead(MPUREG_GYRO_YOUT_L);
  gyroY = (float)((byte_H<<8)| byte_L);
  // Read GyroZ
  byte_H = spiRead(MPUREG_GYRO_ZOUT_H);
  byte_L = spiRead(MPUREG_GYRO_ZOUT_L);
  gyroZ = (float)((byte_H<<8)| byte_L);
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

//
//Computes the dot product of two vectors
//
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;
  
  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

//
//Computes the cross product of two vectors
//
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//
//Multiply the vector by a scalar
//
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

//
//adds two vecotrs
//
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}
