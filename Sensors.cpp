#include "Sensors.h"

Sensors::Sensors() : gps(), compass(), mpu(), barometer(), singleWire() {}

void Sensors::init() {
  gps.init();
  compass.init();
  mpu.init();
  barometer.init();
  singleWire.init();
  
  // calibrate any sensors that need calibration
  for (int i = 0; i < CALIBRATION_ROUNDS; i++) {
    delay(1000);
    mpu.calibrate(i);
    singleWire.calibrate(i);
  }
  
  // init variables
  lastUpdateTime = micros();
  
  // generate initial rotation matrix
  float gyro[3];
  float accel[3];
  float mag[3];
  boolean gotValues = false;
  
  do {
    delay(200);
    mpu.dataInt();
    
    if (mpu.readRawValues(gyro, accel, true) && compass.readRawValues(mag)) {
      
      for (int i = 0; i++; i < 3) {
        sensorData.gyro_b[i] = gyro[i];  
      }
      
      // convert accels and mag to unit vectors
      matrixUnit(accel);
      matrixUnit(mag);
      
      // we want the positive K vector, so invert gravity
      accel[0] = -accel[0];
      accel[1] = -accel[1];
      accel[2] = -accel[2];
      
      // adjust mag to account for downward component of earth's magnetic field
      float scalar = matrixDot(mag, accel);
      for (int i = 0; i < 3; i++) {
        mag[i] -= scalar * accel[i];  
      }
      matrixUnit(mag);
      
      // calculate K x I = J
      float j[3];
      matrixCross(accel, mag, j);
      
      rotation[0][0] = mag[0];
      rotation[0][1] = mag[1];
      rotation[0][2] = mag[2];
      
      rotation[1][0] = j[0];
      rotation[1][1] = j[1];
      rotation[1][2] = j[2];
      
      rotation[2][0] = accel[0];
      rotation[2][1] = accel[1];
      rotation[2][2] = accel[2];
    
      gotValues = true;
    }  
  } while (gotValues == false);
}

//
// update - gets latest data from sensors if it is time to read them again
//
void Sensors::update() {
  // gps
  gps.update();
  
  // barometer
  float temp, pressure;
  if (barometer.readRawValues(&temp, &pressure)) {
    // calculate altitude
    sensorData.pressAltitude = 44330 * (1.0 - pow(pressure / PRESSURE_SEA_LEVEL, 0.1903));
 
  }
  
  // battery
  sensorData.battVoltage = singleWire.readBattery();
    
  // accel/mag
  float gyro[3], accel[3], mag[3];
  if (mpu.readRawValues(gyro, accel, true) && compass.readRawValues(mag)) {
    for (int i = 0; i++; i < 3) {
      sensorData.gyro_b[i] = gyro[i];  
    }
    
    updateRotationMatrix(gyro, accel, mag);
    eulerAngles();
  }

  // airspeed
  float airspeedBody[3] = {singleWire.readAirspeed(), 0, 0};
  sensorData.airspeedRaw = airspeedBody[0];
  matrixRotate(airspeedBody, sensorData.airspeed_e); 
}

//
// updateRotationMatrix - read the accel/mag/gyro, calculate the change in the angle, and update the rotation matrix
//
void Sensors::updateRotationMatrix(float* gyro, float* accel, float* mag) {
      
  //
  // calculate angular change from gyros
  //
      
  // figure out how long it's been since our last update
  unsigned long curTime = micros();
  float timeDelta = curTime - lastUpdateTime;
  lastUpdateTime = curTime;
      
  // calculate angular change from gyro values
  for (int i = 0; i < 3; i++) {
    gyro[i] *= timeDelta / 1000000.0f * DEG2RAD; 
  } 
    
  //
  // calculate angular change from each of mag and accel
  //
    
  // convert mag and accel to unit vectors
  matrixUnit(mag); 
  matrixUnit(accel); 
    
  // invert accel so gravity is correctly pointing down
  for (int i = 0; i < 3; i++) {
    accel[i] = -1.0f * accel[i];
  }
  
  // adjust mag to account for downward component of earth's magnetic field
  float scalar = matrixDot(mag, accel);
  for (int i = 0; i < 3; i++) {
    mag[i] -= scalar * accel[i];  
  }
  matrixUnit(mag);
    
  // calculate angular change from mag and accel
  for (int i = 0; i < 3; i++) {
    mag[i] = mag[i] - rotation[0][i];       
    accel[i] = accel[i] - rotation[2][i];
  }
      
  float magAngle[3];
  matrixCross(rotation[0], mag, magAngle);
  float accelAngle[3];
  matrixCross(rotation[2], accel, accelAngle);
    
  //
  // weight gyros, mag, and accel to calculate final answer
  //
  float finalAngle[3];
  for (int i = 0; i < 3; i++) {
    finalAngle[i] = gyro[i] * GYRO_WEIGHT + magAngle[i] * MAG_WEIGHT + accelAngle[i] * ACCEL_WEIGHT;  
  }
     
  //
  // update rotation matrix with new values
  //
    
  // calculate new vectors
  float I[3], K[3];
  matrixCross(finalAngle, rotation[0], I);
  matrixCross(finalAngle, rotation[2], K);
    
  // update the rotation matrix
  for (int i = 0; i < 3; i++) {
    rotation[0][i] += I[i];
    rotation[2][i] += K[i];    
  }
    
  //
  // check that the rotation matrix vectors are still perpendicular and unit length
  //
  float error = matrixDot(rotation[0], rotation[2]) / 2;
  float temp[3] = {rotation[0][0], rotation[0][1], rotation[0][2]};
    
  for (int i = 0; i < 3; i++) {
    rotation[0][i] = rotation[0][i] - error * rotation[2][i];
    rotation[2][i] = rotation[2][i] - error * temp[i];  
  }
    
  matrixUnit(rotation[0]);
  matrixUnit(rotation[2]);
      
  //
  // calculate the J vector and fill in the last row of the rotation matrix
  //
  matrixCross(rotation[2], rotation[0], rotation[1]);
}

//
// eulerAngles - calculates new euler angles from the rotation matrix
//
void Sensors::eulerAngles() {
  // calulate updated Euler angles
  sensorData.pitch_e = asin(rotation[2][0]);
  sensorData.yaw_e = atan2(rotation[1][0]/cos(sensorData.pitch_e), rotation[0][0]/cos(sensorData.pitch_e)) * RAD2DEG;
  sensorData.roll_e = -atan2(rotation[2][1]/cos(sensorData.pitch_e), rotation[2][2]/cos(sensorData.pitch_e)) * RAD2DEG;
    
  // fixup euler angles
  sensorData.pitch_e = sensorData.pitch_e * RAD2DEG;
    
  if (sensorData.yaw_e < 0)
    sensorData.yaw_e += 360;
      
  if (sensorData.roll_e < 0)
    sensorData.roll_e += 360;   
}

void Sensors::mpuDataInt() {
  mpu.dataInt();  
}

//
// matrixDot - returns the dot product of a . b
//
float Sensors::matrixDot(float* a, float* b) {
  float returnValue = 0;
  
  for (int i = 0; i < 3; i++) {
    returnValue += a[i] * b[i];
  }  
  return(returnValue);
}

//
// matrixUnit - takes vector a and converts it into a unit vector
//
void Sensors::matrixUnit(float* a) {
  float magnitude = 0;
 
  for (int i = 0; i < 3; i++) {
    magnitude += a[i] * a[i];  
  } 
  
  magnitude = sqrt(magnitude);
  
  for (int i = 0; i < 3; i++) {
    a[i] = a[i] / magnitude;  
  } 
}

//
// matrixCross - calculate the cross product of a x b = c
//
void Sensors::matrixCross(float* a, float* b, float* c) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
} 

void Sensors::matrixRotate(float* bodyVec, float* earthVec) {
  for (int i = 0; i < 3; i++) {
    earthVec[i] = rotation[i][0] * bodyVec[0] + rotation[i][1] * bodyVec[1] + rotation[i][2] * bodyVec[2];
  }  
}
