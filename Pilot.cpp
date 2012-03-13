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
  updateThrottleControl();
  updateYawControl();
  updatePitchControl();
  updateRollControl();
  lastUpdateTime = curUpdateTime;
}

void Pilot::updateThrottleControl() {
  switch(captData.curState) {
                
    case STATE_RECOVER:
      pilotData.throttleValue = THROTTLE_MAX;
      break;

    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
      pilotData.throttleValue = throttleMaintainCruiseAirSpeed();
      break;
      
    case STATE_START:
    case STATE_GLIDE:
    case STATE_END:
      pilotData.throttleValue = 0;
      break;
      
    // ?  
    default:
      break;
  }    
}

void Pilot::updateYawControl() {
  switch(captData.curState) {
    
    case STATE_START:
    case STATE_END:
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
      pilotData.yawValue = rudderMaintainBearing();
      break;
      
    case STATE_RECOVER:
    case STATE_GLIDE:
    default:
      pilotData.yawValue = YAW_CENTER_ANGLE;
      break;
  }
}


//
// updatePitchControl - updates elevator value for controller
//
void Pilot::updatePitchControl() {
  switch(captData.curState) {
    
    case STATE_START:
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
      pilotData.pitchValue = elevatorMaintainCruiseAltitude();
      break;
      
    case STATE_RECOVER:
    case STATE_GLIDE:
    case STATE_END:
    default:
      pilotData.pitchValue = PITCH_CENTER_ANGLE;
      break;
  }  
}


//
// updateRollControl - updates aileron value for controller
//
void Pilot::updateRollControl() {
  switch(captData.curState) {
    
    case STATE_START:
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
    case STATE_RECOVER:
    case STATE_GLIDE:
    case STATE_END:
    default:
      pilotData.rollValue = ROLL_CENTER_ANGLE;
      break;
  }  
}


//
// manage the throttle value to captData.deltaAirSpeed
//
float Pilot::throttleMaintainCruiseAirSpeed() {
  float maxIncr =  (THROTTLE_MAX - THROTTLE_MIN) * THROTTLE_MAX_RATE / 100 * dt / 1000;
  float deltaAirSpeed = CRUISE_AIR_SPEED - sensorData.airspeed[0];
  
  float throttleDelta = fmap(abs(deltaAirSpeed),0,MAX_AIR_SPEED-MIN_AIR_SPEED,THROTTLE_MIN,THROTTLE_MAX);
  throttleDelta = (deltaAirSpeed > 0.0) ? throttleDelta : -throttleDelta;
  throttleDelta = constrain(throttleDelta, -maxIncr, maxIncr);

  return(constrain(pilotData.throttleValue+throttleDelta, THROTTLE_MIN, THROTTLE_MAX));
}


//
// rudderMaintainBearing - control the rudder linearly by delta to desired navigation bearing
//
float Pilot::rudderMaintainBearing() {
  float deltaBearing = calcMinimumAngle(navData.estGroundSpeed.direction,navData.curDistance.direction);
  return(YAW_CENTER_ANGLE + clipMechanicalAngle(deltaBearing,YAW_MECHANICAL_MAX) * YAW_SERVO_POLARITY);
}


//
// manage the throttle value to captData.deltaAirSpeed
//
float Pilot::elevatorMaintainCruiseAltitude() {
//  float deltaAltitude = CRUISE_ALTITUDE - sensorData.pressAltitude;
//  float pitchDelta = fmap(abs(deltaAltitude),0,CRUISE_ALTITUDE_THRESHOLD,0,PITCH_MAX);
//  pitchDelta = (deltaAltitude > 0.0) ? pitchDelta : -pitchDelta;
//
//  return(constrain(pitchDelta, PITCH_CENTER_ANGLE-PITCH_MAX, PITCH_CENTER_ANGLE+PITCH_MAX));

  return(sensorData.pitch);

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


//
// calculates a minimum angle between bearings. The result is always between -179 and 180 degrees
//
float Pilot::calcMinimumAngle(float curBearing, float targBearing) {
  float deltaBearingAngle = targBearing - curBearing;
  
  if(deltaBearingAngle > 180) {
    deltaBearingAngle -= 360;
  }
  if(deltaBearingAngle < -179) {
    deltaBearingAngle += 360;
  }
  return deltaBearingAngle;
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
