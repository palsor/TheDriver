#include "Navigator.h"

Navigator::Navigator() {
  destIdx = 0;
  maxValidIdx = -1;
}

//
// getBearingToDest - determines the current destination, and then determines the bearing to that destination
//
float Navigator::getBearingToDest(Waypoint* curLocation) {
  
  // fill in the code to determine the destination, whether we've arrived there, if so what the new destination is, and then determine the bearing of that destination from our current location
  updateDestination(curLocation);
  return(curLocation->bearingToTarget(&track[destIdx]));
}

void Navigator::updateDestination(Waypoint* curLocation) {
  float distance = track[destIdx].distanceToTarget(curLocation);
  
  if(distance <= ARRIVED_THRESHOLD) {
    if(destIdx < maxValidIdx) {
      destIdx += 1;
    }
  }  
  
  #if (NAV_DEBUG > 0)
    softSerial.print("destIdx: ");
    softSerial.print(destIdx);
    softSerial.print("   distance: ");
    softSerial.print(distance);
    softSerial.print("   ");
    track[destIdx].print();
  #endif
}

void Navigator::addTrackWaypoint(float lat, float lon) {
  if(maxValidIdx <= MAX_WAYPOINTS) {
    maxValidIdx += 1;
  } else {
    Serial.println("WARN: MAX_WAYPOINTS exceeded. Overwriting last waypoint");   
  }
  
  track[maxValidIdx].setLocation(lat,lon);
}
