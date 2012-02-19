#include "Pilot.h"

Pilot::Pilot() {
 
}

void Pilot::init() {
  maxThrottleRate = MAX_THROTTLE_RATE / 1000;
}

//
// update plane controls based on desired navigation
//
void Pilot::update() {
  curUpdateTime = millis();
  updateSpeedControl();
  updateHeadingControl();
  lastUpdateTime = curUpdateTime;
  if(PILOT_DEBUG > 0) {
    Serial.print("PIL yawValue ");
    Serial.println(pilotData.yawValue,DEC); 
  }
}

void Pilot::updateSpeedControl() {
  switch(navData.curNavState) {
                
    case NAV_STATE_RECOVER:  // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A
      pilotData.throttleValue = 99;
      break;

    case NAV_STATE_TAKEOFF:  // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
    case NAV_STATE_CLIMB:  // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
    case NAV_STATE_NAVIGATE:  // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
      pilotData.throttleValue = 0;
      break;
      
    case NAV_STATE_START:  // T: 0 P/Y/R: Centered
    case NAV_STATE_GLIDE:  // T: 0, P: minAirSpeed, Y: upWind, R: N/A  
    case NAV_STATE_END:  // T: 0, P/Y/R: Centered
      pilotData.throttleValue = 0;
      break;
      
    // ?  
    default:
      break;
  }    
}

void Pilot::updateHeadingControl() {
  switch(navData.curNavState) {
    
    case NAV_STATE_START:  // T: 0 P/Y/R: Centered
    case NAV_STATE_END:  // T: 0, P/Y/R: Centered  
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
//    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
//    case NAV_STATE_TAKEOFF:
//      pilotData.pitchValue = PITCH_CENTER_ANGLE;
//      pilotData.yawValue = YAW_CENTER_ANGLE;
//      pilotData.rollValue = ROLL_CENTER_ANGLE;
//      break;
//      
//    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
//    case NAV_STATE_CLIMB:
//      pilotData.pitchValue = PITCH_CENTER_ANGLE;
//      pilotData.yawValue = YAW_CENTER_ANGLE;
//      pilotData.rollValue = ROLL_CENTER_ANGLE;
//      break;
      
    case NAV_STATE_TAKEOFF:
    case NAV_STATE_CLIMB:
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case NAV_STATE_NAVIGATE:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE + clipMechanicalAngle(navData.deltaBearing,YAW_MECHANICAL_MAX) * REVERSE_YAW_SERVO;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A  
    case NAV_STATE_RECOVER:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // T: 0, P: minAirSpeed, Y: upWind, R: N/A
    case NAV_STATE_GLIDE:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // ?  
    default:
      break;
  }
}

 float Pilot::manageThrottleRate() {
//   unsigned long deltaTime = curUpdateTime - lastUpdateTime;
//   float requestedThrottleIncr = abs(navData.deltaSpeed) / 1000;
//   float throttleIncr = min(requestedThrottleIncr,maxThrottleIncr) * deltaTime;
//   pilotData.throttleValue = (navData.deltaSpeed > 0) ? pilotData.throttleValue+=throttleIncr : pilotData.throttleValue-=throttleIncr;
 }

float Pilot::clipMechanicalAngle(float angle, int mechMax) {
  if(angle > 0) {
    return(min(mechMax,angle));
  } else {
    return(max(-mechMax,angle));
  }
}
