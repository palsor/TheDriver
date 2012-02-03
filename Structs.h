#ifndef STRUCTS_H
#define STRUCTS_H

struct Waypoint {
  float latitude;
  float longitude;
};

struct Vector {
  float direction;
  float magnitude;
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
  float airSpeed;
};

struct NavData {
  Vector curDistance;  // holds distance/bearing to next waypoint
  Vector curGroundSpeed;  // holds gpsSpeed/gpsBearing
  Vector curAirSpeed;  // holds airspeed/magBearing
  Vector curWindSpeed;  // calculated from groundSpeed - airSpeed
  Vector targetAirSpeed;  // desired heading in plane reference
  float deltaAirSpeed;  // speed change fed to Pilot
  float deltaAltitude;  // altitude change fed to Pilot
  float deltaBearing;  // bearing change fed to Pilot
};

struct PilotData {
  float throttleValue;
  float pitchValue;
  float yawValue;
  float rollValue;
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
  unsigned long navErrors;
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
