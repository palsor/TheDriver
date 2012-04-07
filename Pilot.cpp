#include "Pilot.h"

Pilot::Pilot() {
 
}

void Pilot::init() {
  pilotData.throttleValue = 0.0;
  pilotData.rudderAngle = RUDDER_CENTER_ANGLE;
  pilotData.elevatorAngle = ELEVATOR_CENTER_ANGLE;
  pilotData.aileronAngle = AILERON_CENTER_ANGLE;
  rudderSweepDir = true;
  elevatorSweepDir = true;
  aileronSweepDir = true;
  speedRange = MAX_AIR_SPEED - MIN_AIR_SPEED;
  throttleRange = THROTTLE_MAX - THROTTLE_MIN;
  curUpdateTime = millis();
  lastUpdateTime = millis();
  pilotData.adaptivePitchTrim = 0.0;
  resetVasIntegrator();
}

//
// update plane controls based on desired navigation
//
void Pilot::update() {
  curUpdateTime = millis();
  dt = curUpdateTime - lastUpdateTime;
  updateThrottleControl();
  updateRudderControl();
  calibratePitch();  // adaptive pitch
  updateElevatorControl();
  updateAileronControl();
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
      
    case STATE_INIT:
    case STATE_START:
    case STATE_GLIDE:
    case STATE_END:
      pilotData.throttleValue = 0.0;
      break;
      
    // ?  
    default:
      break;
  }    
}

void Pilot::updateRudderControl() {
  switch(captData.curState) {
    
    case STATE_START:
    case STATE_END:
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
      pilotData.rudderAngle = rudderMaintainBearing();
      break;
      
    case STATE_INIT:
    case STATE_RECOVER:
    case STATE_GLIDE:
    default:
      pilotData.rudderAngle = RUDDER_CENTER_ANGLE;
      break;
  }
}


//
// updateElevatorControl - updates elevator value for controller
//
void Pilot::updateElevatorControl() {
  switch(captData.curState) {
    
    case STATE_START:
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
      pilotData.elevatorAngle = elevatorMaintainCruiseAltitude();
      break;
      
    case STATE_INIT:
    case STATE_RECOVER:
    case STATE_GLIDE:
    case STATE_END:
    default:
      pilotData.elevatorAngle = ELEVATOR_CENTER_ANGLE;
      break;
  }  
}


//
// updateAileronControl - updates aileron value for controller
//
void Pilot::updateAileronControl() {
  switch(captData.curState) {
    
    case STATE_INIT:
    case STATE_START:
    case STATE_TAKEOFF:
    case STATE_CLIMB:
    case STATE_NAVIGATE:
    case STATE_RECOVER:
    case STATE_GLIDE:
    case STATE_END:
    default:
      pilotData.aileronAngle = AILERON_CENTER_ANGLE;
      break;
  }  
}


//
// manage the throttle value to air speed
//
float Pilot::throttleMaintainCruiseAirSpeed() {
  float curThrot =  pilotData.throttleValue;
  float maxIncr =  (THROTTLE_MAX - THROTTLE_MIN) * THROTTLE_MAX_RATE / 100.0 * dt / 1000.0;
  
  float deltaAirSpeed = CRUISE_AIR_SPEED - sensorData.airspeedRaw;
  deltaAirSpeed = constrain(deltaAirSpeed,-speedRange,speedRange);
  
  float throttleDelta = fmap(deltaAirSpeed,-speedRange,speedRange,-throttleRange,throttleRange);
  float c = constrain(throttleDelta,-maxIncr,maxIncr);
  curThrot = (float)curThrot + (float)c;

  return(constrain(curThrot,THROTTLE_MIN,THROTTLE_MAX));
}


//
// rudderMaintainBearing - control the rudder linearly by delta to desired navigation bearing
//
float Pilot::rudderMaintainBearing() {

  
  // bearing to rate
  float deltaBearing_e = calcMinimumAngle(navData.estGroundSpeed.direction,navData.curDistance.direction);
  deltaBearing_e = constrain(deltaBearing_e,-CONTROL_BEARING_THRESHOLD,CONTROL_BEARING_THRESHOLD);
  float deltaBearing_translated = deltaBearing_e * cos(DEG2RAD*sensorData.roll_e);  // Scales deltaPitch_e from 0-100% and inverts if needed based on roll_e
  deltaBearing_translated = constrain(deltaBearing_translated,-CONTROL_BEARING_THRESHOLD,CONTROL_BEARING_THRESHOLD);
  float controlYawRate_b = fmap(deltaBearing_translated,-CONTROL_BEARING_THRESHOLD,CONTROL_BEARING_THRESHOLD,-MAX_YAW_RATE,MAX_YAW_RATE);
  
  // rate to rudder
  float maxIncr_b =  MAX_RUDDER_SERVO_RATE * dt / 1000.0;
 
  float deltaYawRate_b = controlYawRate_b - sensorData.gyro_b[2];  // gyro_b[2] is rate about the z axis (yaw)
  deltaYawRate_b = constrain(deltaYawRate_b,-MAX_YAW_RATE,MAX_YAW_RATE);
  float deltaRudderAngle = fmap(deltaYawRate_b,-MAX_YAW_RATE,MAX_YAW_RATE,-maxIncr_b,maxIncr_b);

  float newRudderAngle = pilotData.rudderAngle + (deltaRudderAngle * RUDDER_SERVO_POLARITY);
  return(constrain(newRudderAngle,-RUDDER_MECHANICAL_MAX,RUDDER_MECHANICAL_MAX)); 
  
  // linear drive deltaBearing to 0
//  return(RUDDER_CENTER_ANGLE + clipMechanicalAngle(deltaBearing_e,RUDDER_MECHANICAL_MAX) * RUDDER_SERVO_POLARITY);
}


//
// manage the throttle value to maintain altitude
//
float Pilot::elevatorMaintainCruiseAltitude() {
  float tmpElev = pilotData.elevatorAngle;
  
  // altitude to pitch
  float deltaAltitude = sensorData.pressAltitude - CRUISE_ALTITUDE;
  deltaAltitude = constrain(deltaAltitude,-CONTROL_ALTITUDE_THRESHOLD,CONTROL_ALTITUDE_THRESHOLD);
  float controlPitch_e = fmap(deltaAltitude,-CONTROL_ALTITUDE_THRESHOLD,CONTROL_ALTITUDE_THRESHOLD,-MAX_PITCH_ANGLE,MAX_PITCH_ANGLE);
  
  // pitch to rate
//  float deltaPitch_e = controlPitch_e - sensorData.pitch_e + pilotData.adaptivePitchTrim;
  float deltaPitch_e = controlPitch_e - sensorData.pitch_e;
  float deltaPitch_translated = deltaPitch_e / zeroLimitCos(sensorData.roll_e,60.0); // Scales deltaPitch_e from 100-200% and inverts if needed based on roll_e
  deltaPitch_translated = constrain(deltaPitch_translated,-MAX_PITCH_ANGLE,MAX_PITCH_ANGLE);
  float controlPitchRate_b = fmap(deltaPitch_translated,-MAX_PITCH_ANGLE,MAX_PITCH_ANGLE,-MAX_PITCH_RATE,MAX_PITCH_RATE);
  
  // rate to elevator
  float maxIncr_b =  MAX_ELEVATOR_SERVO_RATE * dt / 1000.0;
 
  float deltaPitchRate_b = controlPitchRate_b - sensorData.gyro_b[1];  // gyro_b[1] is rate about the y axis (pitch)
  deltaPitchRate_b = constrain(deltaPitchRate_b,-MAX_PITCH_RATE,MAX_PITCH_RATE);
  float deltaElevatorAngle = fmap(deltaPitchRate_b,-MAX_PITCH_RATE,MAX_PITCH_RATE,-maxIncr_b,maxIncr_b);
  float newElevatorAngle = tmpElev + (deltaElevatorAngle * ELEVATOR_SERVO_POLARITY);

  return(constrain(newElevatorAngle,ELEVATOR_CENTER_ANGLE-ELEVATOR_MECHANICAL_MAX,ELEVATOR_CENTER_ANGLE+ELEVATOR_MECHANICAL_MAX));
  
  // linear drive pitch to 0
  //return(ELEVATOR_CENTER_ANGLE-constrain(sensorData.pitch_e,-MAX_PITCH_ANGLE,MAX_PITCH_ANGLE));

}


//
// resetVasIntegrator - reset values and time for vas integrator
//
void Pilot::resetVasIntegrator() {
  vasAltAccum = 0.0;
  startPalt = sensorData.pressAltitude;
  dtAccum = 0.0;
}


//
// integrateVas - altitude integrator using vertical airspeed & time
//
void Pilot::integrateVas() {
  vasAltAccum += sensorData.airspeed_e[2] * dt;
  dtAccum += dt;
}


//
// calibratePitch - adaptive pitch algorithm to offset errors from sensors
//
void Pilot::calibratePitch() {
  integrateVas();

  float curPressDeltaAlt = sensorData.pressAltitude - startPalt;
  if(curPressDeltaAlt - vasAltAccum >= CONTROL_ALTITUDE_THRESHOLD) {
    pilotData.adaptivePitchTrim -= ADAPTIVE_PITCH_MAX_INCREMENT * (ADAPTIVE_PITCH_TIMEOUT - dtAccum) / ADAPTIVE_PITCH_TIMEOUT;
    resetVasIntegrator();
  }
  else if(curPressDeltaAlt - vasAltAccum <= -CONTROL_ALTITUDE_THRESHOLD) {
    pilotData.adaptivePitchTrim += ADAPTIVE_PITCH_MAX_INCREMENT * (ADAPTIVE_PITCH_TIMEOUT - dtAccum) / ADAPTIVE_PITCH_TIMEOUT;
    resetVasIntegrator();
  }
  if(dtAccum >= ADAPTIVE_PITCH_TIMEOUT) {
    resetVasIntegrator();
  }
  pilotData.adaptivePitchTrim = constrain(pilotData.adaptivePitchTrim,-ADAPTIVE_PITCH_LIMIT,-ADAPTIVE_PITCH_LIMIT);
}


//
// sweepControls - diagnostic/test sweep of controls
//
void Pilot::sweepControls() {
  pilotData.throttleValue = 0;
  
  if(pilotData.rudderAngle >= RUDDER_CENTER_ANGLE+RUDDER_MECHANICAL_MAX) {
    rudderSweepDir = false;
  }
  if(pilotData.rudderAngle <= RUDDER_CENTER_ANGLE-RUDDER_MECHANICAL_MAX) {
    rudderSweepDir = true;
  }
  
  pilotData.rudderAngle = (rudderSweepDir) ? pilotData.rudderAngle+1 : pilotData.rudderAngle-1;

  if(pilotData.elevatorAngle >= ELEVATOR_CENTER_ANGLE+ELEVATOR_MECHANICAL_MAX) {
    elevatorSweepDir = false;
  }
  if(pilotData.elevatorAngle <= ELEVATOR_CENTER_ANGLE-ELEVATOR_MECHANICAL_MAX) {
    elevatorSweepDir = true;
  }
  
  pilotData.elevatorAngle = (elevatorSweepDir) ? pilotData.elevatorAngle+1 : pilotData.elevatorAngle-1;
  
  if(pilotData.aileronAngle >= AILERON_CENTER_ANGLE+AILERON_MECHANICAL_MAX) {
    aileronSweepDir = false;
  }
  if(pilotData.aileronAngle <= AILERON_CENTER_ANGLE-AILERON_MECHANICAL_MAX) {
    aileronSweepDir = true;
  }
  
  pilotData.aileronAngle = (aileronSweepDir) ? pilotData.aileronAngle+1 : pilotData.aileronAngle-1;
  
}


//
//
//
float Pilot::zeroLimitCos(float angle, float range) {
  float limVal;
  float halfRange = range/2;
  
  if(angle > (90.0-halfRange) && angle <= 90.0) {
    limVal = cos(DEG2RAD*(90.0-halfRange));
  }
  else if(angle > 90.0 && angle < (90.0+halfRange)) {
    limVal = cos(DEG2RAD*(90.0+halfRange));
  }
  else if(angle > (270.0-halfRange) && angle <= 270.0) {
    limVal = cos(DEG2RAD*(270.0-halfRange));
  }
  else if(angle > (270.0-halfRange) && angle < (270.0+halfRange)) {
    limVal = cos(DEG2RAD*(270.0+halfRange));
  } else {
    limVal = cos(DEG2RAD*angle);
  }
  
  return(limVal);
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
  
  if(deltaBearingAngle > 180.0) {
    deltaBearingAngle -= 360.0;
  }
  if(deltaBearingAngle < -179.0) {
    deltaBearingAngle += 360.0;
  }
  return deltaBearingAngle;
}
 
