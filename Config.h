#ifndef CONFIG_H
#define CONFIG_H

// program flow
#define WAIT_FOR_GPS_LOCK 0

// pins
//#define THROTTLE_SERVO_PIN
//#define PITCH_SERVO_PIN
#define YAW_SERVO_PIN 9
//#define ROLL_SERVO_PIN

#define MPU_SS_PIN 4
#define GPS_MUX_PIN 7
#define MINI_SS_PIN 10

// output config
#define SPI_TX_DELAY 5 // us
#define SERIAL_RATE 38400 // baud

// debug config
//#define SIMULATION_MODE
#define NAV_DEBUG 0
#define CONTROLLER_DEBUG 0 
#define COMPASS_DEBUG 0
#define GPS_DEBUG 0

// compass settings
#define MAG_DECLINATION -6

// Navigator controls
#define MAX_WAYPOINTS 7
#define HOLD_PATTERN_WAYPOINTS 4
#define HOLD_PATTERN_RADIUS 0.001
#define NAV_STATE_END -3
#define NAV_STATE_GLIDE -2
#define NAV_STATE_RECOVER -1
#define NAV_STATE_START 0
#define START_DURATION 3000 // ms
#define NAV_STATE_TAKEOFF 1
#define TAKEOFF_DURATION 3000  // ms
#define NAV_STATE_CLIMB 2
#define NAV_STATE_NAVIGATE 3
#define INVALID_NAV -1
#define ARRIVED_THRESHOLD 0.0001

// dynamics
#define MIN_AIR_SPEED 0
#define CRUISE_AIR_SPEED 0
#define CLIMB_PITCH 15
#define RECOVER_PITCH -15

// mechanical controls
#define PITCH_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define PITCH_MECHANICAL_MAX 25  // mechanical limits of servo travel
#define YAW_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define YAW_MECHANICAL_MAX 25  // mechanical limits of servo travel
#define ROLL_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define ROLL_MECHANICAL_MAX 25  // mechanical limits of servo travel


#endif
