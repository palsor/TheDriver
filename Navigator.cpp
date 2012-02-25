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
  maxValidCourseIdx = INVALID_NAV;
  curHoldIdx = INVALID_NAV;
  maxValidHoldIdx = INVALID_NAV;
  navData.estLocation.latitude = INVALID_NAV;
  navData.estLocation.latitude = INVALID_NAV;
  navData.curNavState = NAV_STATE_END;
  navData.prevNavState = NAV_STATE_END;
  navData.lastStateTransitionTime = millis();
  minAirSpeed = MIN_AIR_SPEED;
  cruiseAirSpeed = CRUISE_AIR_SPEED;
  estDLatAccum = 0.0;
  estDLonAccum = 0.0;
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
}


//
// checks that valid waypoints exist and prepares final course information
//
void Navigator::beginNavigation() {
  if(maxValidCourseIdx != INVALID_NAV) {
    calcHoldPattern(sensorData.curLocation);
    calcCourseDistance();
    navData.estLocation = sensorData.curLocation;  // update estimated location with actual gps fix location
    updateSpeedVectors();
    curCourseIdx = 0;
    curHoldIdx = 0;
    updateDistanceVectors();
    transitionState(NAV_STATE_START);
  } else {  // no valid waypoints added to course
    transitionState(NAV_STATE_END);
  }
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
  for(int i=1;i <=maxValidCourseIdx; i++) {
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
  manageCourse();  // manages distances/waypoints
  updateState();  // naviagtion state machine
  calcPilotInputs();  // updates deltaAirSpeed, deltaAltitude, deltaBearing
  navData.lastUpdateTime = curUpdateTime; // saves timestamp for last run
}


//
// updates distance to destination and manges next waypoint
//
void Navigator::manageCourse() {
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
// updates current ground/air speeds
//
void Navigator::updateSpeedVectors() {
  navData.curAirSpeed.direction = sensorData.magBearing;
//  navData.curAirSpeed.magnitude = sensorData.airSpeed;  // need sensor attached and read
  navData.curAirSpeed.magnitude = sensorData.gpsSpeed;  // PM remove
//  navData.curAirSpeed.magnitude = 40.0;  // PM remove

  navData.curGroundSpeed.direction = sensorData.gpsBearing;  
  navData.curGroundSpeed.magnitude = sensorData.gpsSpeed;

}

//
// resets accumulators and navData.estLocation using sensorData.curLocation
//
void Navigator::resetEstLocation() {
  navData.estLocation = sensorData.curLocation;  // update estimated location with actual gps fix location
  navData.estGroundSpeed = navData.curGroundSpeed;  // reset estimated groundSpeed with gps data
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
  if((navData.curGroundSpeed.magnitude > 0.0) && (navData.curWindSpeed.magnitude > 0.0)) {
    subv(&navData.curWindSpeed,navData.curGroundSpeed,navData.curAirSpeed);
  }
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
      
//    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
//    case NAV_STATE_TAKEOFF:
//      navData.deltaAirSpeed = 0;
//      navData.deltaAltitude = 0;
//      navData.deltaBearing = 0;
//      break;
//      
//    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
//    case NAV_STATE_CLIMB:
//      navData.deltaAirSpeed = 0;
//      navData.deltaAltitude = 0;
//      navData.deltaBearing = 0;
//      break;
      
    case NAV_STATE_TAKEOFF:
    case NAV_STATE_CLIMB:
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case NAV_STATE_NAVIGATE:
      navData.deltaAirSpeed = CRUISE_AIR_SPEED - sensorData.airSpeed;  // cruiseAirSpeed - sensorData.airSpeed
      navData.deltaAltitude = CRUISE_ALTITUDE - sensorData.gpsAltitude;  // cruiseAltitude - sensorData.pressAltitude
      navData.deltaBearing = calcMinimumAngle((sensorData.gpsUpdated == true) ? navData.curGroundSpeed.direction : navData.estGroundSpeed.direction,navData.curDistance.direction);
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
  // 1 knot = 1.85200 kilometers/hr
  // 1 knots = 0.000514444444 kilometers / second  // NOTE: The Google conversion answer is WRONG!!!
  navData.estDistance.direction = navData.estGroundSpeed.direction;
//  navData.estDistance.magnitude = navData.estGroundSpeed.magnitude * 0.00014998800096 * (curUpdateTime - navData.lastUpdateTime) / 1000;
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

/*

  if(NAV_DEBUG > 0) {
    Serial.print("MaxValidCourseIdx ");
    Serial.println(maxValidCourseIdx,DEC);
  
    for(int i=0;i<=maxValidCourseIdx;i++) {
      Serial.print("C ");
      Serial.print(i,DEC);
      Serial.print("\t");
      Serial.print(course[i].latitude,DEC);
      Serial.print("\t");
      Serial.print(course[i].longitude,DEC);
      Serial.print("\t");
      Serial.print(courseDistance[i].magnitude,DEC);
      Serial.print("\t");
      Serial.print(courseDistance[i].direction,DEC);
      Serial.print("\n");
    }

    for(int i=0;i<=maxValidHoldIdx;i++) {
      Serial.print("H ");
      Serial.print(i,DEC);
      Serial.print("\t");
      Serial.print(hold[i].latitude,DEC);
      Serial.print("\t");
      Serial.print(hold[i].longitude,DEC);
      Serial.print("\t");
      Serial.print(holdDistance[i].magnitude,DEC);
      Serial.print("\t");
      Serial.print(holdDistance[i].direction,DEC);
      Serial.print("\n");
    }
  }

//  if(NAV_DEBUG > 3) {
//    Serial.print("addv ");
//    Serial.print(v1.direction,DEC);
//    Serial.print("/");
//    Serial.print(v1DirRad,DEC);
//    Serial.print(" ");
//    Serial.print(v2.direction,DEC);
//    Serial.print("/");
//    Serial.print(v2DirRad,DEC);
//    Serial.print(" ");
//    Serial.print(x,DEC);
//    Serial.print(",");
//    Serial.println(y,DEC);
//  }

//      if(NAV_DEBUG == -1) {
//        Serial.print("NAV deltaBearing ");
//        Serial.print(navData.deltaBearing); 
//        Serial.print(" EGS ");
//        Serial.print(navData.estGroundSpeed.direction,DEC); 
//        Serial.print(" CD ");
//        Serial.println(navData.curDistance.direction,DEC); 
//      }

//  if(NAV_DEBUG > 2) {
//    Serial.print("WS ");
//    Serial.print(navData.curWindSpeed.magnitude,DEC);
//    Serial.print("/");
//    Serial.println(navData.curWindSpeed.direction,DEC);
//  }  

//  if(navData.estDistance.magnitude <= 0.0) { return; };
////where ln is natural log and % is modulo, Δlon is taking shortest route (<180°), and R is the earth’s radius
//  float d = navData.estDistance.magnitude / EARTH_RADIUS;
//  float brng = convDegreesToRadians(navData.estDistance.direction);
//  float lat1 = navData.estLocation.latitude;
//  float lon1 = navData.estLocation.longitude;
//  
////var dLat = d*Math.cos(brng);
//  float dLat = d * cos(brng);
////var lat2 = lat1 + dLat;
//  float lat2 = lat1 + dLat;
////var dPhi = Math.log(Math.tan(lat2/2+Math.PI/4)/Math.tan(lat1/2+Math.PI/4));
//  float dPhi = calcDPhi(lat1,lat2);
////var q = (!isNaN(dLat/dPhi)) ? dLat/dPhi : Math.cos(lat1);  // E-W line gives dPhi=0
//  float q = calcQ(dPhi, dLat, lat1);
////var dLon = d*Math.sin(brng)/q;
//  float dLon = d * sin(brng)/q;
////// check for some daft bugger going past the pole, normalise latitude if so
////if (Math.abs(lat2) > Math.PI/2) lat2 = lat2>0 ? Math.PI-lat2 : -(Math.PI-lat2);
////  if (abs(lat2) > 3.14159265358979323846/2) {
////    lat2 = lat2>0 ? 3.14159265358979323846-lat2 : -(3.14159265358979323846-lat2);
////  }
////lon2 = (lon1+dLon+Math.PI)%(2*Math.PI) - Math.PI;
////  float lon2 = fmod((lon1+dLon+3.14159265358979323846),(2*3.14159265358979323846)) - 3.14159265358979323846;
//  estDLatAccum += dLat;
//  estDLonAccum += dLon;
//  navData.estLocation.latitude =  sensorData.curLocation.latitude + estDLatAccum;
//  navData.estLocation.longitude = sensorData.curLocation.longitude + estDLonAccum;
  
  if(NAV_DEBUG > 1) {
    Serial.print("EGS ");
    Serial.print(navData.estGroundSpeed.magnitude,DEC);
    Serial.print("/");
    Serial.print(navData.estGroundSpeed.direction,DEC);
    Serial.print(" EDIST ");
    Serial.print(navData.estDistance.magnitude,DEC);
    Serial.print("/");
    Serial.print(navData.estDistance.direction,DEC);
    Serial.print(" ELOC ");
    Serial.print(navData.estLocation.latitude,DEC);
    Serial.print(",");
    Serial.print(navData.estLocation.longitude,DEC);
    Serial.print(" DT ");
    Serial.print(curUpdateTime - navData.lastUpdateTime,DEC);
//    Serial.print(" DLAT ");
//    Serial.print(dLat,DEC);
//    Serial.print(" DLON ");
//    Serial.print(dLon,DEC);
    Serial.print(" DACC ");
    Serial.print(estDLatAccum,DEC);
    Serial.print(",");
    Serial.println(estDLonAccum,DEC);
  }
//  if(NAV_DEBUG == -2) {
//    Serial.print(navData.estLocation.latitude,DEC);
//    Serial.print("\t");
//    Serial.print(navData.estLocation.longitude,DEC);
//    Serial.print("\tELOC\n");
//  }

  if(sensorData.gpsUpdated == true && NAV_DEBUG > 0) {
    Serial.print("deltaBearing ");
    Serial.println(navData.deltaBearing,DEC); 
  }
  
      
      if(NAV_DEBUG > 0) {
        Serial.print((sensorData.gpsUpdated==true) ? "DIST " : "EDIST ");
        Serial.print(navData.curDistance.magnitude,DEC);
        Serial.print("/");
        Serial.print(navData.curDistance.direction,DEC);
        Serial.print(" ");
      }
    
  if(NAV_DEBUG == -2) {
    if(sensorData.gpsUpdated == true) {
       Serial.print(sensorData.curLocation.latitude,DEC);
       Serial.print("\t");
       Serial.print(sensorData.curLocation.longitude,DEC);
       Serial.print("\tsmall_blue\n");
    } else {
       Serial.print(navData.estLocation.latitude,DEC);
       Serial.print("\t");
       Serial.print(navData.estLocation.longitude,DEC);
       Serial.print("\tsmall_yellow\n"); 
    }
  }

  if(sensorData.gpsUpdated == true && NAV_DEBUG > 0) {
     Serial.print("NAV ");
     Serial.print((navSelect) ? "C" : "H");
     Serial.print((navSelect) ? curCourseIdx : curHoldIdx, DEC);
     Serial.print(" ");
     Serial.print(navData.curDistance.magnitude,DEC);
     Serial.print("/");
     Serial.print(navData.curDistance.direction,DEC);
     Serial.print(" CL ");
     Serial.print(sensorData.curLocation.latitude,DEC);
     Serial.print(",");
     Serial.print(sensorData.curLocation.longitude,DEC);
     Serial.print(" ALT ");
     Serial.print(sensorData.gpsAltitude,DEC);
     Serial.print(" FIX ");
     Serial.print(sensorData.gpsFixType,DEC);
     Serial.print(" SAT ");
     Serial.println(sensorData.gpsSatellites,DEC);
      Serial.print("CUR GS ");
      Serial.print(navData.curGroundSpeed.magnitude,DEC);
      Serial.print("/");
      Serial.print(navData.curGroundSpeed.direction,DEC);
      Serial.print(" AS ");
      Serial.print(navData.curAirSpeed.magnitude,DEC);
      Serial.print("/");
      Serial.print(navData.curAirSpeed.direction,DEC);
      Serial.print(" WS ");
      Serial.print(navData.curWindSpeed.magnitude,DEC);
      Serial.print("/");
      Serial.println(navData.curWindSpeed.direction,DEC);
  }
  
    if(NAV_DEBUG > 0) {
      Serial.println();
      Serial.print("ERROR ");
      Serial.print(navData.estLocation.latitude - sensorData.curLocation.latitude,DEC);
      Serial.print(",");
      Serial.print(navData.estLocation.longitude - sensorData.curLocation.longitude,DEC);
      Serial.print(" ACCUM ");
      Serial.print(estDLatAccum,DEC);
      Serial.print(",");
      Serial.print(estDLonAccum,DEC);
      Serial.print(" EL ");
      Serial.print(navData.estLocation.latitude,DEC);
      Serial.print(",");
      Serial.print(navData.estLocation.longitude,DEC);
      Serial.print(" DT ");
      Serial.print(curUpdateTime - navData.lastUpdateTime,DEC);
      Serial.print(" HDOP ");
      Serial.println(sensorData.gpsHDOP,DEC);
    }
    
    if(NAV_DEBUG > 0) {
      Serial.print("Advance waypoint: navSelect ");
      Serial.print(navSelect, DEC);
      Serial.print(" ? ");
      Serial.print(curCourseIdx,DEC);
      Serial.print("/");
      Serial.print(maxValidCourseIdx,DEC);
      Serial.print(" : ");
      Serial.print(curHoldIdx,DEC);
      Serial.print("/");
      Serial.println(maxValidHoldIdx,DEC);
    }

  
  if(NAV_DEBUG > 0) {
    Serial.print("MaxValidCourseIdx");
    Serial.println(maxValidCourseIdx,DEC);
  }
  
  if(NAV_DEBUG > 0) {
    Serial.print("State ");
    Serial.print(navData.curNavState,DEC);
    Serial.print("->");
    Serial.println(newState,DEC);
  }
*/
