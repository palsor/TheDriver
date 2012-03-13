#include "Pilot.h"

Pilot::Pilot() {
 
}

void Pilot::init() {

}

//
// update plane controls based on desired navigation
//
void Pilot::update() {
  curUpdateTime = millis();
  dt = curUpdateTime - lastUpdateTime;
  updateSpeedControl();
  updateHeadingControl();
  lastUpdateTime = curUpdateTime;
}

void Pilot::updateSpeedControl() {
  switch(captData.curState) {
                
    case STATE_RECOVER:  // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A
      pilotData.throttleValue = THROTTLE_MAX;
      break;

    case STATE_TAKEOFF:  // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
    case STATE_CLIMB:  // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
    case STATE_NAVIGATE:  // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
      pilotData.throttleValue = maintainCruiseAirSpeed();
      break;
      
    case STATE_START:  // T: 0 P/Y/R: Centered
    case STATE_GLIDE:  // T: 0, P: minAirSpeed, Y: upWind, R: N/A  
    case STATE_END:  // T: 0, P/Y/R: Centered
      pilotData.throttleValue = 0;
      break;
      
    // ?  
    default:
      break;
  }    
}

void Pilot::updateHeadingControl() {
  switch(captData.curState) {
    
    case STATE_START:  // T: 0 P/Y/R: Centered
    case STATE_END:  // T: 0, P/Y/R: Centered  
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
//    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: hold takeoff heading, R: N/A
//    case STATE_TAKEOFF:
//      pilotData.pitchValue = PITCH_CENTER_ANGLE;
//      pilotData.yawValue = YAW_CENTER_ANGLE;
//      pilotData.rollValue = ROLL_CENTER_ANGLE;
//      break;
//      
//    // T: cruiseAirSpeed, P: CLIMB_PITCH, Y: upWind, R: N/A
//    case STATE_CLIMB:
//      pilotData.pitchValue = PITCH_CENTER_ANGLE;
//      pilotData.yawValue = YAW_CENTER_ANGLE;
//      pilotData.rollValue = ROLL_CENTER_ANGLE;
//      break;
      
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    // T: cruiseAirSpeed, P: cruiseAltitude, Y: deltaBearing, R: N/A
    case STATE_NAVIGATE:
      pilotData.pitchValue =  maintainCruiseAltitude();
//      pilotData.yawValue = YAW_CENTER_ANGLE + clipMechanicalAngle(captData.deltaBearing,YAW_MECHANICAL_MAX) * YAW_SERVO_POLARITY;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // T: max, P: RECOVER_PITCH, Y: upWind, R: N/A  
    case STATE_RECOVER:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // T: 0, P: minAirSpeed, Y: upWind, R: N/A
    case STATE_GLIDE:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      pilotData.yawValue = YAW_CENTER_ANGLE;
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
      
    // ?  
    default:
      break;
  }
}


//
// manage the throttle value to captData.deltaAirSpeed
//
float Pilot::maintainCruiseAirSpeed() {
  float maxIncr =  (THROTTLE_MAX - THROTTLE_MIN) * THROTTLE_MAX_RATE / 100 * dt / 1000;
  
//  float throttleDelta = fmap(abs(captData.deltaAirSpeed),0,MAX_AIR_SPEED-MIN_AIR_SPEED,THROTTLE_MIN,THROTTLE_MAX);
//  throttleDelta = (captData.deltaAirSpeed > 0.0) ? throttleDelta : -throttleDelta;
//  throttleDelta = constrain(throttleDelta, -maxIncr, maxIncr);
//
//  return(constrain(pilotData.throttleValue+throttleDelta, THROTTLE_MIN, THROTTLE_MAX));
}


//
// manage the throttle value to captData.deltaAirSpeed
//
float Pilot::maintainCruiseAltitude() {
  
//  float pitchDelta = fmap(abs(captData.deltaAltitude),0,CRUISE_ALTITUDE_THRESHOLD,0,PITCH_MAX);
//  pitchDelta = (captData.deltaAirSpeed > 0.0) ? pitchDelta : -pitchDelta;
//
//  return(constrain(pilotData.pitchValue+pitchDelta, PITCH_CENTER_ANGLE-PITCH_MAX, PITCH_CENTER_ANGLE+PITCH_MAX));
}


//
// 
//
float Pilot::clipMechanicalAngle(float angle, int mechMax) {
  return(constrain(angle,(float)-mechMax,(float)mechMax));
}


//
// float version of the map function
//
float Pilot::fmap(float x, float min1, float max1, float min2, float max2) {
  // y = mx + b
  float m = (max2-min2)/(max1-min1);
  float b = min2 - min1*m;
  return(m*x + b);
} 

/*

  if(PILOT_DEBUG > 0) {
    Serial.print("PIL yawValue ");
    Serial.println(pilotData.yawValue,DEC); 
  }
  
  if(PILOT_DEBUG > 0) {
    Serial.print("airSpeed ");
    Serial.print(sensorData.airSpeed,DEC);
    
    Serial.print(" deltaAirSpeed ");
    Serial.print(captData.deltaAirSpeed,DEC); 
    
    Serial.print(" throttleDelta ");
    Serial.print(throttleDelta,DEC);
    
    Serial.print(" maxIncr ");
    Serial.print(maxIncr, DEC);
    
    Serial.print(" throttleValue ");
    Serial.println(pilotData.throttleValue,DEC);
  }
  
  
  if(PILOT_DEBUG > 0) {
    Serial.print("altitude ");
    Serial.print(sensorData.gpsAltitude,DEC);
    
    Serial.print(" deltaAltitude ");
    Serial.print(captData.deltaAltitude,DEC); 
    
    Serial.print(" pitchDelta ");
    Serial.print(pitchDelta,DEC);
    
    Serial.print(" pitchValue ");
    Serial.println(pilotData.pitchValue,DEC);
  }  
*/
