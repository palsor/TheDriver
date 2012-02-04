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
  int navState;  // current state of navigation state machine
  int navPrevState;  // previous state of navigation state machine
  unsigned long lastStateTransitionTime;  // last navigator state transition time
  Vector curDistance;  // calculated distance/bearing to next waypoint
  Vector curGroundSpeed;  // created from gpsSpeed/gpsBearing
  Vector curAirSpeed;  // created from airspeed/magBearing
  Vector curWindSpeed;  // calculated from curGroundSpeed - curAirSpeed
  Vector targetAirSpeed;  // calculated desired heading in plane reference with normalized speed
  float deltaAirSpeed;  // speed change fed to Pilot
  float deltaAltitude;  // altitude change fed to Pilot
  float deltaBearing;  // bearing change fed to Pilot
};

struct PilotData {
  float throttleValue;  // throttle value sent to controller
  float pitchValue;  // pitch value sent to controller
  float yawValue;  // yaw value sent to controller
  float rollValue;  // roll value sent to controller
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
