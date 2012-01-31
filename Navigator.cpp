#include "Navigator.h"

Navigator::Navigator() {
  destIdx = 0;
  maxValidIdx = -1;
}

//
// update - calculate new nav data
//
void Navigator::update() {
  
}

//
// getBearingToDest - determines the current destination, and then determines the bearing to that destination
//
float Navigator::getBearingToDest() {
  
  // fill in the code to determine the destination, whether we've arrived there, if so what the new destination is, and then determine the bearing of that destination from our current location
  updateDestination();
  //return(curLocation->bearingToTarget(&track[destIdx]));
  return 0;
}

void Navigator::updateDestination() {
  /*float distance = track[destIdx].distanceToTarget(curLocation);
  
  if(distance <= ARRIVED_THRESHOLD) {
    if(destIdx < maxValidIdx) {
      destIdx += 1;
    }
  } */ 
}

void Navigator::addTrackWaypoint(float lat, float lon) {
  /*if(maxValidIdx <= MAX_WAYPOINTS) {
    maxValidIdx += 1;
  } else {
    errorData.navWaypointError = true;   
  }
  
  track[maxValidIdx].setLocation(lat,lon);*/
}
