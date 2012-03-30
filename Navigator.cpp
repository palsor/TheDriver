#include "Navigator.h"

Navigator::Navigator() {
  course = (Waypoint*)malloc(sizeof(Waypoint) * (MAX_WAYPOINTS)); // Array of waypoints that form the course
  courseDistance = (Vector*)malloc(sizeof(Vector) * (MAX_WAYPOINTS));  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]

  hold = (Waypoint*)malloc(sizeof(Waypoint) * (HOLD_PATTERN_WAYPOINTS));  // Array of waypoints that create a holding pattern course around the course origin
  holdDistance = (Vector*)malloc(sizeof(Vector) * (HOLD_PATTERN_WAYPOINTS));  // Array of vectors (distance/bearing) between waypoints. Index i is waypoint[i-1]->waypoint[i]
}

//
// initializes navigation indexes to known invalid states
//
void Navigator::init() {
  navSelect = true;  // nav to course
  curCourseIdx = INVALID_NAV;
  navData.maxValidCourseIdx = INVALID_NAV;
  curHoldIdx = INVALID_NAV;
  maxValidHoldIdx = INVALID_NAV;
  navData.estLocation.latitude = INVALID_NAV;
  navData.estLocation.latitude = INVALID_NAV;
  minAirSpeed = MIN_AIR_SPEED;
  cruiseAirSpeed = CRUISE_AIR_SPEED;
  estDLatAccum = 0.0;
  estDLonAccum = 0.0;
}


//
// creates new entry in the course and courseDistance arrays using the supplied lat/lon
//
void Navigator::addWaypoint(float lat, float lon) {
  if(navData.maxValidCourseIdx < MAX_WAYPOINTS - 1) {
    navData.maxValidCourseIdx += 1;
    course[navData.maxValidCourseIdx].latitude = lat;
    course[navData.maxValidCourseIdx].longitude = lon;
  } else {
    errorData.navWaypointError = true;   
  }
}


//
// checks that valid waypoints exist and prepares final course information
//
void Navigator::beginNavigation() {
  calcHoldPattern(sensorData.curLocation);
  calcCourseDistance();
  navData.estLocation = sensorData.curLocation;  // update estimated location with actual gps fix location
  updateSpeedVectors();
  curCourseIdx = 0;
  curHoldIdx = 0;
  updateDistanceVectors();
}


//
// creates the hold and holdDistance arrays
//
void Navigator::calcHoldPattern(Waypoint w) {
  for(int i=0;i<HOLD_PATTERN_WAYPOINTS;i++) {
    maxValidHoldIdx = i;
    float calcAngle = 2 * 3.14159265358979323846 * i / HOLD_PATTERN_WAYPOINTS;
    hold[maxValidHoldIdx].latitude = w.latitude + calcDecDLat(HOLD_PATTERN_RADIUS * cos(calcAngle));
    hold[maxValidHoldIdx].longitude = w.longitude + calcDecDLon(HOLD_PATTERN_RADIUS * sin(calcAngle), w.latitude);
  }
}


//
// populates courseDistance and calcDistance vectors from waypoints 
//
void Navigator::calcCourseDistance() {  
  for(int i=1;i <=navData.maxValidCourseIdx; i++) {
    calcDistanceVecFromWaypoints(&courseDistance[i],course[i-1],course[i]);
  }
  
  calcDistanceVecFromWaypoints(&holdDistance[0],hold[maxValidHoldIdx],hold[0]);
  for(int i=1;i <=maxValidHoldIdx; i++) {
    calcDistanceVecFromWaypoints(&holdDistance[i],hold[i-1],hold[i]);
  }
}


//
// update navigation calculations
//
void Navigator::update() {
  curUpdateTime = millis();  // sets timestamp for this run
  updateSpeedVectors();  // updates cur air/ground speed with sensor data

  if(sensorData.gpsUpdated == true) {  // if gps updated this iteration

    calcCurWindSpeed();  // re-calculate curWindSpeed
    resetEstLocation();  // reset estimator
    
  } else {  // gps not updated this iteration
    updateEstLocation();  // updates navData.estLocation
  }

  updateDistanceVectors();  // updates curDistance
  
  while(advanceWaypoint()) {  // advance to the next waypoint if we've arrived at the current one
      if(navSelect == true) {
        if(curCourseIdx==navData.maxValidCourseIdx) {  // last course waypoint reached; switch to hold pattern
          navSelect = false;
        } else {
          curCourseIdx+=1;
        }
      } else {
        curHoldIdx+=1;
        curHoldIdx%=maxValidHoldIdx;  // loop through hold pattern waypoints indefinitely
      }
      
      updateDistanceVectors();  // updates curDistance for navSelect ? course waypoint : hold waypoint

  }
  
  navData.lastUpdateTime = curUpdateTime; // saves timestamp for last run
}


//
// updates current ground/air speeds
//
void Navigator::updateSpeedVectors() {
  navData.curAirSpeed.direction = sensorData.yaw_e;
  navData.curAirSpeed.magnitude = sqrt(sensorData.airspeed_e[0] * sensorData.airspeed_e[0] + sensorData.airspeed_e[1] * sensorData.airspeed_e[1]);

  navData.curGroundSpeed.direction = sensorData.gpsBearing;  
  navData.curGroundSpeed.magnitude = sensorData.gpsSpeed;

}

//
// resets accumulators and navData.estLocation using sensorData.curLocation
//
void Navigator::resetEstLocation() {
  navData.estLocation = sensorData.curLocation;  // update estimated location with actual gps fix location
  navData.estGroundSpeed = (navData.curWindSpeed.magnitude > 0.0) ? navData.curGroundSpeed : navData.curAirSpeed;
  estDLatAccum = 0.0;  // zero estimated location accumulators
  estDLonAccum = 0.0;  // zero estimated location accumulators
}


//
// updates navData.estLocation
//
void Navigator::updateEstLocation() {
  calcEstGroundSpeed();  // estGroundSpeed =  curAirSpeed + curWindSpeed (curWindSpeed @ last gps fix)
  calcEstDistance();  // translates speed & time into distance
  float estDistDirRad = convDegreesToRadians(navData.estDistance.direction);
  
  if(navData.estDistance.magnitude > 0.0) {
    estDLatAccum += calcDecDLat(navData.estDistance.magnitude * cos(estDistDirRad));
    navData.estLocation.latitude =  sensorData.curLocation.latitude + estDLatAccum;
    
    estDLonAccum += calcDecDLon(navData.estDistance.magnitude * sin(estDistDirRad), navData.estLocation.latitude);
    navData.estLocation.longitude = sensorData.curLocation.longitude + estDLonAccum;
  }
}


//
// updates curDistance
//
void Navigator::updateDistanceVectors() {
  if(navSelect == true) {  // use course waypoints
    if(sensorData.gpsUpdated == true) {
      calcDistanceVecFromWaypoints(&navData.curDistance,sensorData.curLocation,course[curCourseIdx]);   
    } else {
      calcDistanceVecFromWaypoints(&navData.curDistance,navData.estLocation,course[curCourseIdx]);
    }
  } else {  // use hold pattern waypoints
    if(sensorData.gpsUpdated == true) {
      calcDistanceVecFromWaypoints(&navData.curDistance,sensorData.curLocation,hold[curHoldIdx]);
    } else {
      calcDistanceVecFromWaypoints(&navData.curDistance,navData.estLocation,hold[curHoldIdx]);
    }
  }
}


//
// checks if navigation should advance to the next waypoint (true=arrived/advance false=continue navigation)
//
boolean Navigator::advanceWaypoint() {
  if(navData.curDistance.magnitude <= ARRIVED_THRESHOLD) { return(true); };
  return(false);
}


//
// calculates curWindSpeed vector
//
void Navigator::calcCurWindSpeed() {
  if((navData.curGroundSpeed.magnitude > 0.0) && (navData.curAirSpeed.magnitude > 0.0)) {
    subv(&navData.curWindSpeed,navData.curGroundSpeed,navData.curAirSpeed);
  }
}


//
// returns a vector calculated from from waypoint 1 to waypoint 2 in *v
//
void Navigator::calcDistanceVecFromWaypoints(Vector* v, Waypoint w1, Waypoint w2) {
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
  float gpsDistanceToDestination = sqrt(dLat*dLat + q*q*dLon*dLon) * EARTH_RADIUS;
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
// radians to degress conversion
//
float Navigator::convRadiansToDegrees(float radian) {
  return(radian * 180 / 3.14159265358979323846);
}


//
// intermediate term for calcDistanceVector
//
float Navigator::calcDPhi(float lat1, float lat2) {
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


//
// calculate distance vector from airSpeed, windSpeed, and delta time
//
void Navigator::calcEstDistance(){
  navData.estDistance.direction = navData.estGroundSpeed.direction;
  navData.estDistance.magnitude = navData.estGroundSpeed.magnitude / 1000 * (curUpdateTime - navData.lastUpdateTime) / 1000;
}


//
// calculate estimated ground speed vector from airSpeed & windSpeed
//
void Navigator::calcEstGroundSpeed(void) {
  if(navData.curWindSpeed.magnitude > 0.0) {
    addv(&navData.estGroundSpeed, navData.curAirSpeed, navData.curWindSpeed);
  } else {
    navData.estGroundSpeed = navData.curAirSpeed;
  }
}


//
// calcualate incremental decimal lat/long from distance
//
float Navigator::calcDecDLat(float d) {
  return convRadiansToDegrees(d / EARTH_RADIUS);
}


//
// calcualate incremental decimal lat/long from distance
//
float Navigator::calcDecDLon(float d, float lat) {
  float r = EARTH_RADIUS * cos(convDegreesToRadians(lat));  // adjust the earth's radius based on latitude
  return convRadiansToDegrees(d / r);
}

// vector functions

//
// subtract two vectors (v1-v2) and place the result in *v
//
void Navigator::subv(Vector* v, Vector v1, Vector v2){
  if(v1.magnitude == 0) {
    *v = v2;
    return;
  }
  if(v2.magnitude == 0) {
    *v = v1;
    return; 
  }
  
  float v1DirRad = convDegreesToRadians(v1.direction);
  float v2DirRad = convDegreesToRadians(v2.direction);
  float x=v1.magnitude * sin(v1DirRad) - v2.magnitude * sin(v2DirRad);
  float y=v1.magnitude * cos(v1DirRad) - v2.magnitude * cos(v2DirRad);
  
  v->magnitude = hypot(x,y);
  v->direction = -(atan2(y,x) - 3.14159265358979323846 / 2);
  v->direction = convRadiansToDegrees(v->direction);
  if(v->direction > 359.0) { v->direction -= 360.0; };
  if(v->direction < 0.0) { v->direction += 360.0; };
}

//
// add two vectors (v1+v2) and place the result in *v
//
void Navigator::addv(Vector* v, Vector v1, Vector v2){
  if(v1.magnitude == 0) {
    *v = v2;
    return;
  }
  if(v2.magnitude == 0) {
    *v = v1;
    return; 
  }
  
  float v1DirRad = convDegreesToRadians(v1.direction);
  float v2DirRad = convDegreesToRadians(v2.direction);
  float x=v1.magnitude * sin(v1DirRad) + v2.magnitude * sin(v2DirRad);
  float y=v1.magnitude * cos(v1DirRad) + v2.magnitude * cos(v2DirRad);
  
  v->magnitude = hypot(x,y);
  v->direction = -(atan2(y,x) - 3.14159265358979323846 / 2);
  v->direction = convRadiansToDegrees(v->direction);
  if(v->direction > 359) { v->direction -= 360; };
  if(v->direction < 0) { v->direction += 360; };
}

