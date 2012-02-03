#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "Config.h"
#include "Externs.h"

class Navigator {
  public:
    Navigator();
    void init();  // initializes navigation indexes to known invalid states
    void addWaypoint(float lat, float lon);  // creates new entry in the waypoint and courseDistance arrays using the supplied lat/lon
    void beginNavigation();  // checks for valid waypoints and prepares final course information
    void update();   // update navigation calculations
    
  private:
    Waypoint waypoint[MAX_WAYPOINTS]; // Array of waypoint objects that forms course
    Vector courseDistance[MAX_WAYPOINTS];  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i];
    int maxValidNavIdx;  // max valid index of waypoints & courseDistances
    int curNavIdx;  // current index (next destination waypoint and current courseDistance)   
    void updateDistanceVectors();  // updates curDistance
    void updateSpeedVectors();  // updates ground/air/windSpeed
    boolean advanceWaypoint();  // checks if navigation should advance to the next waypoint (true=arrived/advance false=continue navigation)
    void calcPilotInputs();  // calculates deltas between current and desired airSpeed altitude and bearing
    
    // math
    float calcMinimumAngle(float curBearing, float targBearing);  // calculates a minimum angle between bearings. The result is always between -179 and 180 degrees
    float convDegreesToRadians(float degree);  // degrees to radians conversion
    void calcDistanceVector(Vector* v, Waypoint w1, Waypoint w2);  // returns a vector calculated from from waypoint 1 to waypoint 2
      float calcDPhi(float lat1, float lat2);  // intermediate term for calcDistanceVector
      float calcDLon(float lon1, float lon2);  // intermediate term for calcDistanceVector
      float calcQ(float dPhi, float dLat, float lat1);  // intermediate term for calcDistanceVector
    void subv(Vector* v, Vector v1, Vector v2);  // subtract two vectors (v1-v2) and place the result in *v
    void addv(Vector* v, Vector v1, Vector v2);  // add two vectors (v1+v2) and place the result in *v

};

#endif
