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
  unsigned long curUpdateTime;  // timestamp at the start of the current call to navigator.update()
  float estDLatAccum;  // accumulated
  float estDLonAccum;  // accumulated
  Waypoint* course; // Array of waypoints that form the course
  Vector* courseDistance;  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]
  Waypoint* hold;  // Array of waypoints that create a holding pattern course around the course origin
  Vector* holdDistance;  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]
  
    // waypoint and navigation vars
    boolean navSelect;  // true=navigate to course waypoints; false=navigate to hold waypoints
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
    void updateSpeedVectors();  // updates ground/air peed
    void resetEstLocation();  // resets estLocation with curLocation
    void updateEstLocation();  // updates navData.estLocation
    void updateDistanceVectors();  // updates curDistance
    boolean advanceWaypoint();  // checks if navigation should advance to the next waypoint (true=arrived/advance false=continue navigation)
    void calcCurWindSpeed();  // updates curWindSpeed
    
    // math
    float calcMinimumAngle(float curBearing, float targBearing);  // calculates a minimum angle between bearings. The result is always between -179 and 180 degrees
    void calcDistanceVecFromWaypoints(Vector* v, Waypoint w1, Waypoint w2);  // returns a vector calculated from from waypoint 1 to waypoint 2
      float convRadiansToDegrees(float radian);
      float convDegreesToRadians(float degree);  // degrees to radians conversion
      float calcDPhi(float lat1, float lat2);  // intermediate term for calcDistanceVector
      float calcDLon(float lon1, float lon2);  // intermediate term for calcDistanceVector
      float calcQ(float dPhi, float dLat, float lat1);  // intermediate term for calcDistanceVector
    void calcEstGroundSpeed(); // calculate estimated ground speed vector from airSpeed & windSpeed
    void calcEstDistance();  // calculate estimated distance vector using delta time
    float calcDecDLat(float d);  // calcualate incremental decimal lat/long from distance
    float calcDecDLon(float d, float lat);  // calcualate incremental decimal lat/long from distance    
    void subv(Vector* v, Vector v1, Vector v2);  // subtract two vectors (v1-v2) and place the result in *v
    void addv(Vector* v, Vector v1, Vector v2);  // add two vectors (v1+v2) and place the result in *v

};

#endif
