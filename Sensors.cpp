#include "Sensors.h"

Sensors::Sensors() : gps(), compass(), mpu(), barometer(), singleWire() {}

void Sensors::init() {
  gps.init();
  compass.init();
  mpu.init();
  barometer.init();
  singleWire.init();
  
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
    
    if (mpu.readRawValues(gyro, accel) && compass.readRawValues(mag)) {
      
      // convert accels and mag to unit vectors
      matrixUnit(accel);
      matrixUnit(mag);
      
      // we want the positive K vector, so invert gravity
      accel[0] = -accel[0];
      accel[1] = -accel[1];
      accel[2] = -accel[2];
      
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
  // read rotation independent sensors
  gps.update();
  barometer.update();
    
  // update the rotation matrix as well as pitch/yaw/roll
  updateRotationMatrix();

  // read rotation dependent sensors
  updateAirspeed();  
}

//
// readAirspeed - read the differential pressure sensor and calculate airspeed
//
void Sensors::updateAirspeed() {  
  // read airspeed
  float airspeed = singleWire.readAirspeed();
  
  // rotate the airspeed into the earth's frame of reference
  float airspeedBody[3] = {airspeed, 0, 0};
  matrixRotate(airspeedBody, sensorData.airspeed);
}

//
// updateRotationMatrix - read the accel/mag/gyro, calculate the change in the angle, and update the rotation matrix
//
void Sensors::updateRotationMatrix() {
  // read MPU and compass, then perform DCM algorithm
  float gyro[3];
  float accel[3];
  float mag[3];
    
  if (mpu.readRawValues(gyro, accel) && compass.readRawValues(mag)) {
      
    //
    // calculate angular change from gyros
    //
      
    // figure out how long it's been since our last update
    unsigned long curTime = micros();
    float timeDelta = curTime - lastUpdateTime;
    lastUpdateTime = curTime;
      
    // calculate angular change from gyro values
    for (int i = 0; i < 3; i++) {
      gyro[i] *= 32768 / 2000 * timeDelta / 1000000.0f * DEG2RAD; 
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
    // calculate the J vector
    //
    matrixCross(rotation[2], rotation[0], rotation[1]);
      
    // calulate updated Euler angles
    sensorData.pitch = asin(rotation[2][0]);
    sensorData.yaw = atan2(rotation[1][0]/cos(sensorData.pitch), rotation[0][0]/cos(sensorData.pitch)) * RAD2DEG;
    sensorData.roll = -atan2(rotation[2][1]/cos(sensorData.pitch), rotation[2][2]/cos(sensorData.pitch)) * RAD2DEG;
      
    // fixup euler angles
    sensorData.pitch = sensorData.pitch * RAD2DEG;
      
    if (sensorData.yaw < 0)
      sensorData.yaw += 360;
        
    if (sensorData.roll < 0)
      sensorData.roll += 360;
   }    
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

/*
// 
// update - read from the MPU, do any necessary calculations, and update data structures
//
void MPU6000::update() {
  
  // only run our algorithm if there is new data to work from
  if (newdata) {
    newdata = 0; // clear out interrupt flag
    
    float gyroMpu[3] = {0, 0, 0};
    float accelMpu[3] = {0, 0, 0};
    
    // read the latest data out of the MPU
    readRawValues(gyroMpu, accelMpu);     
    
    unsigned long curTime = micros();
    float timeOffset = (float)(curTime - lastUpdateTime) / 1000000;
    lastUpdateTime = curTime;
    
    // calculate angles by scaling the raw values for the calibration offset, register definition and time
    gyroMpu[0] = (gyroMpu[0] - gyroCalibrationOffset[0]) * GYRO_SCALE * timeOffset;
    gyroMpu[1] = (gyroMpu[1] - gyroCalibrationOffset[1]) * GYRO_SCALE * timeOffset;
    gyroMpu[2] = (gyroMpu[2] - gyroCalibrationOffset[2]) * GYRO_SCALE * timeOffset;
    
    // rotate the angles into the earth's frame of reference and convert to radians
    float gyroEarth[3];
    
    for (int i = 0; i < 3; i++) {
      gyroEarth[i] = rotation[i][0] * gyroMpu[0] + rotation[i][1] * gyroMpu[1] + rotation[i][2] * gyroMpu[2];
      gyroEarth[i] *= .0174532925;
    }
    
    float x = gyroEarth[0];
    float y = gyroEarth[1];
    float z = gyroEarth[2];
    
    // build a rotation matrix from the instantaneous angles
    float rotationInst[3][3];

    rotationInst[0][0] = cos(y) * cos(z);
    rotationInst[0][1] = -cos(x) * sin(z) + sin(x) * sin(y) * cos(z);
    rotationInst[0][2] = sin(x) * sin(z) + cos(x) * sin(y) * cos(z);
  
    rotationInst[1][0] = cos(y) * sin(z);
    rotationInst[1][1] = cos(x) * cos(z) + sin(x) * sin(y) * sin(z);
    rotationInst[1][2] = -sin(x) * cos(z) + cos(x) * sin(y) * sin(z);
  
    rotationInst[2][0] = -sin(y);
    rotationInst[2][1] = sin(x) * cos(y);
    rotationInst[2][2] = cos(x) * cos(y);
    
    // add the new instantaneous rotation into the existing rotation
    float temp[3][3];
    
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {     
        temp[i][j] = rotationInst[i][0] * rotation[0][j] + rotationInst[i][1] * rotation[1][j] + rotationInst[i][2] * rotation[2][j];
      }  
    }
    
    // copy our temp data back into our permanent rotation arrays
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        rotation[i][j] = temp[i][j];
      }    
    }
    
    // calulate updated Euler angles
    sensorData.pitch = -asin(rotation[2][0]) / 0.0174532925;
    sensorData.yaw = atan2(rotation[1][0], rotation[0][0]) / 0.0174532925;
    sensorData.roll = atan2(rotation[2][1], rotation[2][2]) / 0.0174532925;
    
    Serial.print(sensorData.pitch);
    Serial.print(" ");
    Serial.print(sensorData.yaw);
    Serial.print(" ");
    Serial.println(sensorData.roll);

  }
}

// 
// from compass
//
// calculate heading
  float tempBearing = atan2(magRaw[1], magRaw[0]);
  
  // make heading always between 0 and 2PI
  if (tempBearing < 0)
    tempBearing += 2 * 3.14159;
  
  // convert heading to degrees and apply magnetic declination  
  tempBearing = (tempBearing * 180 / 3.14159) + MAG_DECLINATION;
  
  if (tempBearing < 0)
    tempBearing += 360;
  
  // copy to data structure
  sensorData.magBearing = tempBearing;
  
  */
