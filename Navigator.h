#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "Waypoint.h"
#include "Config.h"
#include "Externs.h"

class Navigator {
  public:
    Navigator();
    void addTrackWaypoint(float lat, float lon);
    float getBearingToDest(Waypoint* curLocation);
    
  private:
    void updateDestination(Waypoint* curLocation);  // Checks distance and sets destination
    Waypoint track[MAX_WAYPOINTS]; // Array of waypoint objects
    int maxValidIdx;  
    int destIdx;
};

#endif
