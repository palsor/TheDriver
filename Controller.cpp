#include "Controller.h"

//
// constructor
//
Controller::Controller() {}

//
// init - do any required setup
//
void Controller::init() {

//  throttleServo.attach(THROTTLE_SERVO_PIN);
//  throttleServo.write(0);  // init servo to 0

//  pitchServo.attach(PITCH_SERVO_PIN);
//  pitchServo.write(0);  // init servo to 0

  yawServo.attach(YAW_SERVO_PIN);
  yawServo.write(YAW_CENTER_ANGLE);  // init servo to mechanical center

//  rollServo.attach(ROLL_SERVO_PIN);
//  rollServo.write(0);  // init servo to 0


} 

//
// updateSteering - updates the servo for the front wheels, accounting for mechanical limits
//
void Controller::update() {
  applyThrottleValue();
  applyPitchValue();
  applyYawValue();
  applyRollValue();
} 

void Controller::applyThrottleValue() {
//  throttleServo.write(map(round(throttleValue),0,99,0,359);
}

void Controller::applyPitchValue() {
//  pitchServo.write(map(round(pitchValue),0,99,0,359);
}

void Controller::applyYawValue() {
  yawServo.write(map(round(pilotData.yawValue),0,99,0,359));
}

void Controller::applyRollValue() {
//  rollServo.write(map(round(rollValue),0,99,0,359);
}
