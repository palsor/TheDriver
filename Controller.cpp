#include "Controller.h"

//
// constructor
//
Controller::Controller() : steeringServo() {}

//
// init - do any required setup
//
void Controller::init() {
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(X_CENTER_ANGLE);  // init servo to mechanical center
  xTrim = 0;  // init trim to 0
  controlBearing = 0;  // init control bearing
} 

//
// updateSteering - updates the servo for the front wheels, accounting for mechanical limits
//
void Controller::updateSteering(float curMagBearing, float curGpsBearing, float targBearing, float gpsSpeed) {
  
  // calculate bearing to control to
  updateControlBearing(curMagBearing, curGpsBearing, targBearing);
  
  // calculate angle to turn to
  int deltaAngle = (gpsSpeed < GPS_MIN_SPEED_THRESHOLD) ? 0 : calculateDeltaBearingAngle(curGpsBearing, controlBearing);   // note transition from float bearings to int angle for servo control
  
  // clip angle and update servo
  steeringServo.write(X_CENTER_ANGLE + xTrim + clipServoAngle(deltaAngle));
  
  if(CONTROLLER_DEBUG) {
    softSerial.print("TargBearing: ");
    softSerial.print(targBearing);
    softSerial.print("   deltaAngle: ");
    softSerial.print(deltaAngle);
    softSerial.print("   clipServoAngle: ");
    softSerial.println(clipServoAngle(deltaAngle));
  }
} 

//
// updateControlBearing - calculate new controlBearing to hold including damping
//
void Controller::updateControlBearing(float curMagBearing, float curGpsBearing, float targBearing) {
  controlBearing = targBearing;
}

//
// calculateDeltaBearingAngle - get minimum angle (deltaBearingAngle) between targBearing and curBearing deltaBearingAngle must always be between -179 and 180 degrees
//
int Controller::calculateDeltaBearingAngle(float curBearing, float targBearing) {
  int deltaBearingAngle = targBearing - curBearing;
  
  if(deltaBearingAngle > 180) {
    deltaBearingAngle -= 360;
  }
  if(deltaBearingAngle < -179) {
    deltaBearingAngle += 360;
  }
  return deltaBearingAngle;
}

//
// clipServoAngle - apply mechanical limits to servo angle
//
int Controller::clipServoAngle(int angle) {
  if(angle > X_MECHANICAL_MAX) {
    angle = X_MECHANICAL_MAX;
  }
  if(angle < -X_MECHANICAL_MAX) { 
    angle = -X_MECHANICAL_MAX;
  }
  return angle;
}
