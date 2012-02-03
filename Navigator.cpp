#include "Navigator.h"

Navigator::Navigator() {

}

//
// initializes navigation indexes to known invalid states
//
void Navigator::init() {
  curNavIdx = INVALID_NAV_IDX;
  maxValidNavIdx = INVALID_NAV_IDX;
}


//
// creates new entry in the waypoint and courseDistance arrays using the supplied lat/lon
//
void Navigator::addWaypoint(float lat, float lon) {
  if(maxValidNavIdx < MAX_WAYPOINTS) {
    maxValidNavIdx += 1;
    waypoint[maxValidNavIdx].latitude = lat;
    waypoint[maxValidNavIdx].longitude = lon;
    if (maxValidNavIdx==0) { 
      courseDistance[0].direction = USE_CURLOC;
      courseDistance[0].magnitude = USE_CURLOC;
    } else {
      calcDistanceVector(&courseDistance[maxValidNavIdx],waypoint[maxValidNavIdx-1],waypoint[maxValidNavIdx]);
    }
  } else {
    errorData.navWaypointError = true;   
  }
}


//
// checks that valid waypoints exist and prepares final course information
//
void Navigator::beginNavigation() {
  if(maxValidNavIdx != INVALID_NAV_IDX) {  // add home waypoint, init curNavIdx=0
    curNavIdx=0;
    maxValidNavIdx+=1;
    waypoint[maxValidNavIdx] = sensorData.curLocation;
    calcDistanceVector(&courseDistance[maxValidNavIdx],waypoint[maxValidNavIdx-1],waypoint[maxValidNavIdx]);
  } else {  // STOP!!! No valid waypoints defined
     Serial.println("No waypoints");
     
     debugData.navErrors += 1;
     while(1) {
       delay(1000);
     }
  }
  
  for(int i;i<=maxValidNavIdx;i++) {
    Serial.print(i,DEC);
    Serial.print(" ");
    Serial.print(waypoint[i].latitude,DEC);
    Serial.print(" ");
    Serial.print(waypoint[i].longitude,DEC);
    Serial.print(" ");
    Serial.print(courseDistance[i].magnitude,DEC);
    Serial.print(" ");
    Serial.print(courseDistance[i].direction,DEC);
    Serial.println();
  }
}


//
// update navigation calculations
//
void Navigator::update() {
  updateDistanceVectors();  // updates curDistance
  
  while((curNavIdx != maxValidNavIdx) && advanceWaypoint()) {  // advance to the next waypoint if we've arrived at the current one
      curNavIdx+=1;
      updateDistanceVectors();  // updates curDistance for new waypoint
  }
  
  updateSpeedVectors();  // updates curAirSpeed, curGroundSpeed, curWindSpeed
  
  calcPilotInputs();  // updates deltaAirSpeed, deltaAltitude, deltaBearing
}


//
// updates curDistance
//
void Navigator::updateDistanceVectors() {
  calcDistanceVector(&navData.curDistance,sensorData.curLocation,waypoint[curNavIdx]);
}

//
// updates ground/air/windSpeed
//
void Navigator::updateSpeedVectors() {
  navData.curAirSpeed.direction = sensorData.magBearing;
  navData.curAirSpeed.magnitude = sensorData.airSpeed;
  
  navData.curDistance.direction = sensorData.gpsBearing;
  navData.curDistance.magnitude = sensorData.gpsSpeed;
  
  subv(&navData.curWindSpeed,navData.curGroundSpeed,navData.curAirSpeed);
}

//
// checks if navigation should advance to the next waypoint (true=arrived/advance false=continue navigation)
//
boolean Navigator::advanceWaypoint() {
  if(navData.curDistance.magnitude <= ARRIVED_THRESHOLD) {
    if(curNavIdx < maxValidNavIdx) {
      curNavIdx += 1;
      return(true);
    }
  }
  return(false);
}

//
// calculates deltas between current and desired airSpeed altitude and bearing
//
void Navigator::calcPilotInputs() {
  navData.deltaAirSpeed = 0;
  navData.deltaAltitude = 0;  
  navData.deltaBearing = calcMinimumAngle(navData.curAirSpeed.direction,navData.curDistance.direction);
}

//
// returns a vector calculated from from waypoint 1 to waypoint 2 in *v
//
void Navigator::calcDistanceVector(Vector* v, Waypoint w1, Waypoint w2) {
  // convert to radians
  float lat1 = convDegreesToRadians(w1.latitude);
  float lon1 = convDegreesToRadians(w1.longitude);
  float lat2 = convDegreesToRadians(w2.latitude);
  float lon2 = convDegreesToRadians(w2.longitude);
  
  // calculate intermediate terms
  float dPhi = calcDPhi(lat1,lat2);
  float dLat = lat2-lat1;
  float dLon = calcDLon(lon1,lon2);
  float q = calcQ(dPhi, dLat, lat1);
  
  float gpsDestinationBearingRadians = atan2(dLon,dPhi);  
  float gpsDestinationBearingDegrees = gpsDestinationBearingRadians * 180 / 3.14159265358979323846;
  
  // Hack to fix the math (empirically added)
  if(gpsDestinationBearingDegrees < 0) {
    gpsDestinationBearingDegrees += 360;
  }
  v->direction = gpsDestinationBearingDegrees;
  float gpsDistanceToDestination = sqrt(dLat*dLat + q*q*dLon*dLon) * 6371;
  v->magnitude = gpsDistanceToDestination;


}


//
// calculates a minimum angle between bearings. The result is always between -179 and 180 degrees
//
float Navigator::calcMinimumAngle(float curBearing, float targBearing) {
  float deltaBearingAngle = targBearing - curBearing;
  
  if(deltaBearingAngle > 180) {
    deltaBearingAngle -= 360;
  }
  if(deltaBearingAngle < -179) {
    deltaBearingAngle += 360;
  }
  return deltaBearingAngle;
}

//
// degrees to radians conversion
//
float Navigator::convDegreesToRadians(float degree) {
  return(degree * 3.14159265358979323846 / 180);
}

//
// intermediate term for calcDistanceVector
//
float Navigator::calcDPhi(float lat1, float lat2) {
  // ŒîœÜ = ln(tan(lat2/2+œÄ/4)/tan(lat1/2+œÄ/4)) 	[= the ‚Äòstretched‚Äô latitude difference]
  return(log(tan(lat2/2 + 0.78539816339744830962)/tan(lat1/2 + 0.78539816339744830962)));
}

//
// intermediate term for calcDistanceVector
//
float Navigator::calcDLon(float lon1, float lon2) {
  float dLon = lon2-lon1;

  if(dLon > 3.14159265358979323846) {
    dLon = -(2 * 3.14159265358979323846 - dLon);
  }
  if(dLon < -3.14159265358979323846) {
    dLon = (2 * 3.14159265358979323846 + dLon);
  }

  return(dLon);
}

//
// intermediate term for calcDistanceVector
//
float Navigator::calcQ(float dPhi, float dLat, float lat1) {
  float q;
  if(dPhi != 0) {
    q = dLat / dPhi;
  } else {
    q = cos(lat1);
  }
  return(q);
}

// vector functions

//
// subtract two vectors (v1-v2) and place the result in *v
//
void Navigator::subv(Vector* v, Vector v1, Vector v2){
  float x=v1.magnitude * sin(v1.direction) - v2.magnitude * sin(v2.direction);
  float y=v1.magnitude * cos(v1.direction) - v2.magnitude * cos(v2.direction);
  
  v->direction = atan(x/y);
  v->magnitude = hypot(x,y);
}

//
// add two vectors (v1+v2) and place the result in *v
//
void Navigator::addv(Vector* v, Vector v1, Vector v2){
  float x=v1.magnitude * sin(v1.direction) + v2.magnitude * sin(v2.direction);
  float y=v1.magnitude * cos(v1.direction) + v2.magnitude * cos(v2.direction);
  
  v->direction = atan(x/y);
  v->magnitude = hypot(x,y);
}
