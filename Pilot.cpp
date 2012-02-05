#include "Pilot.h"

Pilot::Pilot() {
 
}

void Pilot::init() {
  
}

//
// update plane controls based on desired navigation
//
void Pilot::update() {
  updateSpeedControl();
  updateHeadingControl();
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
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
    case NAV_STATE_TAKEOFF:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
    case NAV_STATE_CLIMB:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case NAV_STATE_NAVIGATE:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = clipMechanicalAngle(navData.deltaBearing,YAW_MECHANICAL_MAX,YAW_CENTER_ANGLE);
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

float Pilot::clipMechanicalAngle(float angle, int mechMax, int centerAngle) {
  if(angle > 0) {
    return(min(centerAngle-mechMax,angle));
  } else {
    return(max(centerAngle+mechMax,angle));
  }
}
