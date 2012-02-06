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
  
  Waypoint* course; // Array of waypoints that form the course
Vector* courseDistance;  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]
Waypoint* hold;  // Array of waypoints that create a holding pattern course around the course origin
Vector* holdDistance;  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]
  
    // waypoint and navigation vars
    boolean navSelect;  // true=navigate to course waypoints; false=navigate to hold waypoints
    int maxValidCourseIdx;  // max valid index of waypoints & courseDistances
    int curCourseIdx;  // current index (next destination waypoint and current courseDistance)   
    int maxValidHoldIdx;  // max valid index of waypoints & courseDistances
    int curHoldIdx;  // current index (next destination waypoint and current courseDistance)   

    // dynamics vars
    float cruiseAirSpeed;
    float minAirSpeed;
    float cruiseAltitude;
    
    // init
    // addWaypoint
    // beginNavigation
      void calcHoldPattern(Waypoint w);  // creates a holdPattern and holdPatternDistance arrays around the supplied Waypoint w
      void calcCourseDistance();  // populates courseDistance vectors from course waypoints
    
    // update
    void manageCourse();  // manages distances and next waypoint
      void updateDistanceVectors();  // updates curDistance
      boolean advanceWaypoint();  // checks if navigation should advance to the next waypoint (true=arrived/advance false=continue navigation)
    void updateSpeedVectors();  // updates ground/air/windSpeed
    void updateState();  // navigation state machine
      boolean priorityStateChecks();  // checks for error conditions to make priority state transitions independent of current state
      void transitionState(int newState);  // transitions state and assocaited variables
    void calcPilotInputs();  // calculates deltas between current and desired airSpeed altitude and bearing
    
    // math
    float calcMinimumAngle(float curBearing, float targBearing);  // calculates a minimum angle between bearings. The result is always between -179 and 180 degrees
    void calcDistanceVector(Vector* v, Waypoint w1, Waypoint w2);  // returns a vector calculated from from waypoint 1 to waypoint 2
      float convDegreesToRadians(float degree);  // degrees to radians conversion
        float calcDPhi(float lat1, float lat2);  // intermediate term for calcDistanceVector
        float calcDLon(float lon1, float lon2);  // intermediate term for calcDistanceVector
        float calcQ(float dPhi, float dLat, float lat1);  // intermediate term for calcDistanceVector
    void subv(Vector* v, Vector v1, Vector v2);  // subtract two vectors (v1-v2) and place the result in *v
    void addv(Vector* v, Vector v1, Vector v2);  // add two vectors (v1+v2) and place the result in *v

};

#endif
