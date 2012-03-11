#include "MPU6000.h"

//
// constructor
//
MPU6000::MPU6000() {}

// 
// init - setup the MPU
//
void MPU6000::init() {    
    
    // configure MPU registers
    spiWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET); // Chip reset
    delay(100);
    
    spiWrite(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ); // Wake up device and select GyroZ clock (better performance)
    spiWrite(MPUREG_USER_CTRL, BIT_I2C_IF_DIS); // Disable I2C bus (recommended on datasheet)    
    spiWrite(MPUREG_SMPLRT_DIV,19);     // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
    spiWrite(MPUREG_CONFIG, BITS_DLPF_CFG_20HZ);  // FS & DLPF   FS=2000º/s, DLPF = 20Hz (low pass filter)
    spiWrite(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale +/-2000º/s
    spiWrite(MPUREG_ACCEL_CONFIG,0x08);            // Accel scale 4g (4096LSB/g)
    spiWrite(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    spiWrite(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
    
    // init values
    gyroCalibrationOffset[0] = 0;
    gyroCalibrationOffset[1] = 0;
    gyroCalibrationOffset[2] = 0;
    newdata = 0;
    
    // calibrate gyros
    float gyroMpu[3];
    float accelMpu[3];
    float tempOffset[3];
    
    for (float i = 1; i <= GYRO_CALIBRATION_ROUNDS; i++) {
      delay(1000);
      newdata++;
      if (readRawValues(gyroMpu, accelMpu)) {
        tempOffset[0] = tempOffset[0] * (1.0f - 1.0f/i) + gyroMpu[0] * 1.0f/i;
        tempOffset[1] = tempOffset[1] * (1.0f - 1.0f/i) + gyroMpu[1] * 1.0f/i;
        tempOffset[2] = tempOffset[2] * (1.0f - 1.0f/i) + gyroMpu[2] * 1.0f/i;
      }
    }
    
    gyroCalibrationOffset[0] = tempOffset[0];
    gyroCalibrationOffset[1] = tempOffset[1];
    gyroCalibrationOffset[2] = tempOffset[2];
}

//
// readRawValues - reads values from the MPU, scales them, and converts the gryo to degrees
//
boolean MPU6000::readRawValues(float* gyro, float* accel) {
  boolean returnValue = false;
  
  if (newdata) {
    int byte_H;
    int byte_L;
  
    // Read AccelX
    byte_H = spiRead(MPUREG_ACCEL_XOUT_H);
    byte_L = spiRead(MPUREG_ACCEL_XOUT_L);
    accel[0] = (float)(((int)byte_H<<8)| byte_L) * ACCEL_X_SIGN;
    // Read AccelY
    byte_H = spiRead(MPUREG_ACCEL_YOUT_H);
    byte_L = spiRead(MPUREG_ACCEL_YOUT_L);
    accel[1] = (float)(((int)byte_H<<8)| byte_L) * ACCEL_Y_SIGN;
    // Read AccelZ
    byte_H = spiRead(MPUREG_ACCEL_ZOUT_H);
    byte_L = spiRead(MPUREG_ACCEL_ZOUT_L);
    accel[2] = (float)(((int)byte_H<<8)| byte_L) * ACCEL_Z_SIGN;
    
    // Read Temp
    // byte_H = spiRead(MPUREG_TEMP_OUT_H);
    // byte_L = spiRead(MPUREG_TEMP_OUT_L);
    // tempRaw = (byte_H<<8)| byte_L; 
     
    // Read GyroX
    byte_H = spiRead(MPUREG_GYRO_XOUT_H);
    byte_L = spiRead(MPUREG_GYRO_XOUT_L);
    gyro[0] = ((float)(((int)byte_H<<8) | byte_L)) * GYRO_X_SIGN - gyroCalibrationOffset[0];
  
    // Read GyroY
    byte_H = spiRead(MPUREG_GYRO_YOUT_H);
    byte_L = spiRead(MPUREG_GYRO_YOUT_L);
    gyro[1] = ((float)(((int)byte_H<<8) | byte_L)) * GYRO_Y_SIGN - gyroCalibrationOffset[1];
    // Read GyroZ
    byte_H = spiRead(MPUREG_GYRO_ZOUT_H);
    byte_L = spiRead(MPUREG_GYRO_ZOUT_L);
    gyro[2] = ((float)(((int)byte_H<<8) | byte_L)) * GYRO_Z_SIGN - gyroCalibrationOffset[2];
    
    newdata = 0;
    returnValue = true;
  }
  
  return(returnValue);
}

//
// MPU6000 utility functions
//
byte MPU6000::spiRead(byte reg) {
  byte dump;
  byte return_value;
  byte addr = reg | 0x80; // Set most significant bit
  digitalWrite(MPU_SS_PIN, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MPU_SS_PIN, HIGH);
  return(return_value);
}

void MPU6000::spiWrite(byte reg, byte data) {
  byte dump;
  digitalWrite(MPU_SS_PIN, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(MPU_SS_PIN, HIGH);
  delay(1);
}

void MPU6000::dataInt() {
  newdata++;
}

