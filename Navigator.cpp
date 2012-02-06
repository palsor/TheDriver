#include "Navigator.h"

Navigator::Navigator() {

}

//
// initializes navigation indexes to known invalid states
//
void Navigator::init() {
  navSelect = true;  // nav to course
  curCourseIdx = INVALID_NAV;
  maxValidCourseIdx = INVALID_NAV;
  curHoldIdx = INVALID_NAV;
  maxValidHoldIdx = INVALID_NAV;
  navData.curNavState = NAV_STATE_END;
  navData.prevNavState = NAV_STATE_END;
  navData.lastStateTransitionTime = millis();
  minAirSpeed = MIN_AIR_SPEED;
  cruiseAirSpeed = CRUISE_AIR_SPEED;
}


//
// creates new entry in the course and courseDistance arrays using the supplied lat/lon
//
void Navigator::addWaypoint(float lat, float lon) {
  if(maxValidCourseIdx < MAX_WAYPOINTS - 1) {
    maxValidCourseIdx += 1;
    course[maxValidCourseIdx].latitude = lat;
    course[maxValidCourseIdx].longitude = lon;
  } else {
    errorData.navWaypointError = true;   
  }
Serial.print("MaxValidCourseIdx ");  // remove
Serial.println(maxValidCourseIdx,DEC);  // remove
}


//
// checks that valid waypoints exist and prepares final course information
//
void Navigator::beginNavigation() {
  if(maxValidCourseIdx != INVALID_NAV) {
    calcHoldPattern(sensorData.curLocation);
    curCourseIdx = 0;
    curHoldIdx = 0;
    transitionState(NAV_STATE_START);
  } else {  // no valid waypoints added to course
    transitionState(NAV_STATE_END);
  }
  
    // remove
    Serial.print("MaxValidCourseIdx ");
    Serial.println(maxValidCourseIdx,DEC);
  
  for(int i=0;i<=maxValidCourseIdx;i++) {
    Serial.print("C ");
    Serial.print(i,DEC);
    Serial.print(" ");
    Serial.print(course[i].latitude,DEC);
    Serial.print("\t");
    Serial.print(course[i].longitude,DEC);
    Serial.print(" ");
    Serial.print(courseDistance[i].magnitude,DEC);
    Serial.print(" ");
    Serial.print(courseDistance[i].direction,DEC);
    Serial.print("\n");
  }
  for(int i=0;i<=maxValidHoldIdx;i++) {
    Serial.print("H ");
    Serial.print(i,DEC);
    Serial.print(" ");
    Serial.print(hold[i].latitude,DEC);
    Serial.print("\t");
    Serial.print(hold[i].longitude,DEC);
    Serial.print(" ");
    Serial.print(holdDistance[i].magnitude,DEC);
    Serial.print(" ");
    Serial.print(holdDistance[i].direction,DEC);
    Serial.print("\n");
  }
}


//
// creates the hold and holdDistance arrays
//
void Navigator::calcHoldPattern(Waypoint w) {
  for(int i=0;i<HOLD_PATTERN_WAYPOINTS;i++) {
    maxValidHoldIdx = i;
    float calcAngle = 2 * 3.14159265358979323846 * i / HOLD_PATTERN_WAYPOINTS;
    hold[maxValidHoldIdx].latitude = w.latitude + HOLD_PATTERN_RADIUS * cos(calcAngle);
    hold[maxValidHoldIdx].longitude = w.longitude + HOLD_PATTERN_RADIUS * sin(calcAngle);
  }
}


//
// populates courseDistance vectors from course waypoints
//
void Navigator::calcCourseDistance() {  
  for(int i=1;i <=maxValidCourseIdx; i++) {
    calcDistanceVector(&courseDistance[i],course[i-1],course[i]);
  }
}


//
// update navigation calculations
//
void Navigator::update() {
  manageCourse();  // manages distances/waypoints
  updateSpeedVectors();  // updates curAirSpeed, curGroundSpeed, curWindSpeed, targAirSpeed
  updateState();  // naviagtion state machine
  calcPilotInputs();  // updates deltaAirSpeed, deltaAltitude, deltaBearing
}


//
// updates distance and manges next waypoint
//
void Navigator::manageCourse() {
  updateDistanceVectors();  // updates curDistance
  
  while(advanceWaypoint()) {  // advance to the next waypoint if we've arrived at the current one
      if(navSelect) {
        if(curCourseIdx==maxValidCourseIdx) {  // last course waypoint reached; switch to hold pattern
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
}

//
// updates curDistance
//
void Navigator::updateDistanceVectors() {
  if(navSelect) {  // use course waypoints
    calcDistanceVector(&navData.curDistance,sensorData.curLocation,course[curCourseIdx]);
  } else {  // use hold pattern waypoints
    calcDistanceVector(&navData.curDistance,sensorData.curLocation,hold[curHoldIdx]);
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
// updates current ground/air/wind speeds & calculates targetAirSpeed
//
void Navigator::updateSpeedVectors() {
  navData.curAirSpeed.direction = sensorData.magBearing;
  navData.curAirSpeed.magnitude = sensorData.airSpeed;
  
  navData.curDistance.direction = sensorData.gpsBearing;
  navData.curDistance.magnitude = sensorData.gpsSpeed;
  
  subv(&navData.curWindSpeed,navData.curGroundSpeed,navData.curAirSpeed);
}


//
// navigation state machine
//
void Navigator::updateState() {
  unsigned long curTime = millis();
  if(priorityStateChecks()) { return; };  // checks for error conditions to make priority state transitions independent of current state  
  
  switch(navData.curNavState) {
    
    // T: 0 P/Y/R: Centered
    case NAV_STATE_START:
      if(curTime-navData.lastStateTransitionTime >= START_DURATION) { transitionState(NAV_STATE_TAKEOFF); };
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
    case NAV_STATE_TAKEOFF:
      if(curTime-navData.lastStateTransitionTime >= TAKEOFF_DURATION) { transitionState(NAV_STATE_CLIMB); };
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
    case NAV_STATE_CLIMB:
      // if(cruiseAltitude >= sensorData.pressAltitude) { transitionState(NAV_STATE_NAVIGATE); };
      transitionState(NAV_STATE_NAVIGATE);
      break;
      
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case NAV_STATE_NAVIGATE:
      // only priorityStateChecks transition state out of NAV_STATE_NAVIGATE
      break;
      
    // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A  
    case NAV_STATE_RECOVER:
      // if(sensorData.airSpeed >= cruiseAirSpeed) { transitionState(NAV_STATE_CLIMB); };
      transitionState(NAV_STATE_CLIMB);
      break;
      
    // T: 0, P: minAirSpeed, Y: upWind, R: N/A
    case NAV_STATE_GLIDE:
      // put in a time check to transition to NAV_STATE_END if no groundSpeed movement for a specified duration
      break;
      
    // T: 0, P/Y/R: Centered  
    case NAV_STATE_END:
      break;
      
    // ?  
    default:
      break;
  }  
}


//
// checks for error conditions to make priority state transitions independent of current state
//
boolean Navigator::priorityStateChecks() {
  // if batt check cutoff -> NAV_STATE_GLIDE
  // if batt check distance -> navSelect=false (next loop iteration switches to hold waypoints)
//  if(sensorData.airSpeed <= minAirSpeed) {
//    if(navData.curNavState != NAV_STATE_RECOVER) { transitionState(NAV_STATE_RECOVER); };
//    return(true);
//  }
  return(false);
}


//
// transitions state and associated variables
//
void Navigator::transitionState(int newState) {
Serial.print("State ");
Serial.print(navData.curNavState,DEC);
Serial.print("->");
Serial.println(newState,DEC);
  navData.prevNavState = navData.curNavState;
  navData.curNavState = newState;
  navData.lastStateTransitionTime = millis();
}


//
// calculates deltas between current and desired airSpeed altitude and bearing
//
void Navigator::calcPilotInputs() {

  switch(navData.curNavState) {
    
    // T: 0 P/Y/R: Centered
    case NAV_STATE_START:
      navData.deltaAirSpeed = 0;
      navData.deltaAltitude = 0;
      navData.deltaBearing = 0;
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
    case NAV_STATE_TAKEOFF:
      navData.deltaAirSpeed = 0;
      navData.deltaAltitude = 0;
      navData.deltaBearing = 0;
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
    case NAV_STATE_CLIMB:
      navData.deltaAirSpeed = 0;
      navData.deltaAltitude = 0;
      navData.deltaBearing = 0;
      break;
      
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case NAV_STATE_NAVIGATE:
      navData.deltaAirSpeed = 0;  // cruiseAirSpeed - sensorData.airSpeed
      navData.deltaAltitude = 0;  // cruiseAltitude - sensorData.pressAltitude
      navData.deltaBearing = calcMinimumAngle(navData.curAirSpeed.direction,navData.curDistance.direction);
      break;
      
    // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A  
    case NAV_STATE_RECOVER:
      navData.deltaAirSpeed = 0;
      navData.deltaAltitude = 0;
      navData.deltaBearing = 0;
      break;
      
    // T: 0, P: minAirSpeed, Y: upWind, R: N/A
    case NAV_STATE_GLIDE:
      navData.deltaAirSpeed = 0;
      navData.deltaAltitude = 0;
      navData.deltaBearing = 0;
      break;
      
    // T: 0, P/Y/R: Centered  
    case NAV_STATE_END:
      navData.deltaAirSpeed = 0;
      navData.deltaAltitude = 0;
      navData.deltaBearing = 0;
      break;
      
    // ?  
    default:
      break;
  }  

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
