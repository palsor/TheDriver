#ifndef STRUCTS_H
#define STRUCTS_H

struct Waypoint {
  float latitude;
  float longitude;
};

struct SensorData {
  float magBearing;   
  float gpsBearing;   
  float gpsSpeed;       
  Waypoint curLocation;
  float hDliution;      
  float vDilution;
  unsigned long fixAge;
  float pitch;   
  float yaw;
  float roll;
  float gyroXRate;
  float gyroYRate;
  float gyroZRate;
  float gpsAltitude;
  float pressAltitude;
  float airspeed;
};

struct NavData {
  float targBearing;
  float targDistance;
  float controlBearing;
  int targPitch;
  int targYaw;
  int targRoll;
  int targThrottle; 
};

struct ErrorData {
  bool compassReadError;
  bool navWaypointError;
};

struct DebugData {
  unsigned long gpsParseErrors;
  unsigned long gpsSentences;
  unsigned long mainLoopIterations;
  unsigned long sensorUpdates;
  unsigned long navUpdates;
  unsigned long sensorAvgDelayTime;
  unsigned long sensorAvgRunTime;
  unsigned long sensorWorstCaseDelayTime;
  unsigned long sensorWorstCaseRunTime;
  unsigned long navAvgDelayTime;
  unsigned long navAvgRunTime;
  unsigned long navWorstCaseDelayTime;
  unsigned long navWorstCaseRunTime;
  unsigned long averageSerialTime;
};

#endif
