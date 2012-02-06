#ifndef EXTERNS_H
#define EXTERNS_H

#include "Structs.h"

extern SensorData sensorData;
extern NavData navData;
extern PilotData pilotData;
extern ErrorData errorData;
extern DebugData debugData;

extern Waypoint course[]; // Array of waypoints that form the course
extern Vector courseDistance[];  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]
extern Waypoint hold[];  // Array of waypoints that create a holding pattern course around the course origin
extern Vector holdDistance[];  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]

#endif
