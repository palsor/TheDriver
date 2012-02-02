#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "Config.h"
#include "Externs.h"

class Navigator {
  public:
    Navigator();
    void update();
    void addTrackWaypoint(float lat, float lon);
    float getBearingToDest();
    
  private:
    void updateDestination();  // Checks distance and sets destination
    Waypoint track[MAX_WAYPOINTS]; // Array of waypoint objects
    int maxValidIdx;  // max valid index of waypoints
    int destIdx;  // current destination index (next waypoint)
};

#endif
