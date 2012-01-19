#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <Arduino.h>

class Waypoint {
  public:
    Waypoint();
    float getLat();
    float getLon();
    void setLocation(float lat, float lon);
    float bearingToTarget(Waypoint* target);
    float distanceToTarget(Waypoint* target);
    void print();
    
  private:
    float convDegreesToRadians(float degree);
    float calcDPhi(float lat1, float lat2);
    float calcDLon(float lon1, float lon2);
    float calcQ(float dPhi, float dLat, float lat1);
    float lat;
    float lon;
};

#endif
