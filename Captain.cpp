#include "Captain.h"

//
// constructor
// 
Captain::Captain() {}

//
// init
//
void Captain::init() {
  captData.curState = STATE_INIT;
  captData.prevState = STATE_INIT;
  captData.lastStateTransitionTime = millis();
}

//
// update
//
void Captain::update() {
  if(priorityStateChecks()) { updateState(); };  // checks for error conditions to make priority state transitions independent of current state
}

//
// captain state machine
//
void Captain::updateState() {
  unsigned long curTime = millis();

  switch(captData.curState) {
    
    // qualified by link test success, valid waypoints, and gps locked in 3D
    case STATE_INIT:
      if(debugData.linkTestSuccess && (navData.maxValidCourseIdx != INVALID_NAV) && ((WAIT_FOR_GPS_LOCK==0) || (sensorData.gpsFixType == 3))) { transitionState(STATE_START); };
      break;
      
    // T: 0 P/Y/R: Centered
    case STATE_START:
      if(curTime-captData.lastStateTransitionTime >= START_DURATION) { transitionState(STATE_TAKEOFF); };
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
    case STATE_TAKEOFF:
      if(curTime-captData.lastStateTransitionTime >= TAKEOFF_DURATION) { transitionState(STATE_CLIMB); };
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
    case STATE_CLIMB:
      // if(cruiseAltitude >= sensorData.pressAltitude) { transitionState(STATE_NAVIGATE); };
      transitionState(STATE_NAVIGATE);
      break;
      
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case STATE_NAVIGATE:
      // only priorityStateChecks transition state out of STATE_NAVIGATE
      break;
      
    // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A  
    case STATE_RECOVER:
      // if(sensorData.airSpeed >= cruiseAirSpeed) { transitionState(STATE_CLIMB); };
      transitionState(STATE_CLIMB);
      break;
      
    // T: 0, P: minAirSpeed, Y: upWind, R: N/A
    case STATE_GLIDE:
      // put in a time check to transition to STATE_END if no groundSpeed movement for a specified duration
      break;
      
    // T: 0, P/Y/R: Centered  
    case STATE_END:
      break;
      
    // ?  
    default:
      break;
  }  
}


//
// checks for error conditions to make priority state transitions independent of current state
//
boolean Captain::priorityStateChecks() {
  // if batt check cutoff -> STATE_GLIDE
  // if batt check distance -> navSelect=false (next loop iteration switches to hold waypoints)
//  if(sensorData.airSpeed <= minAirSpeed) {
//    if(captData.curState != STATE_RECOVER) { transitionState(STATE_RECOVER); };
//    return(true);
//  }
  return(true);
}


//
// transitions state and associated variables
//
void Captain::transitionState(int newState) {
//  Serial.print(captData.curState,DEC);
//  Serial.print("->");
//  Serial.println(newState,DEC);
  captData.prevState = captData.curState;
  captData.curState = newState;
  captData.lastStateTransitionTime = millis();
}

