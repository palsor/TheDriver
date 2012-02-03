#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "Config.h"
#include "Externs.h"

class Navigator {
  public:
    Navigator();
    void init();
    void update();
    void addWaypoint(float lat, float lon);
    void beginNavigation();

    
  private:
    Waypoint waypoint[MAX_WAYPOINTS]; // Array of waypoint objects that forms course
    Vector courseDistance[MAX_WAYPOINTS];  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i]->waypoint[i+1];
    int maxValidNavIdx;  // max valid index of waypoints
    int curNavIdx;  // current waypoint index (next destination)   
    void updateDistanceVectors();
    void updateSpeedVectors();
    boolean advanceWaypoint();
    void calcPilotInputs();
    
    // math
    float calcMinimumAngle(float curBearing, float targBearing);  // calculates a delta angle
    void calcDistanceVector(Vector* v, Waypoint w1, Waypoint w2);
    float convDegreesToRadians(float degree);
    float calcDPhi(float lat1, float lat2);
    float calcDLon(float lon1, float lon2);
    float calcQ(float dPhi, float dLat, float lat1);
    void subv(Vector* v, Vector v1, Vector v2);
    void addv(Vector* v, Vector v1, Vector v2);

};

#endif
