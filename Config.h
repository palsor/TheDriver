#ifndef CONFIG_H
#define CONFIG_H

// program flow
#define WAIT_FOR_GPS_LOCK 0

// pins
//#define THROTTLE_SERVO_PIN
//#define PITCH_SERVO_PIN
#define YAW_SERVO_PIN 9
//#define ROLL_SERVO_PIN

#define GPS_MUX_PIN 7
#define SOFT_SERIAL_RX 11
#define SOFT_SERIAL_TX 12

// output config
#define SERIAL_RATE 38400 // baud
#define SOFT_SERIAL_RATE 9600 // baud

// real time config (millis)
#define COMM_OUTPUT_RATE 1000

// debug config
#define NAV_DEBUG 0
#define CONTROLLER_DEBUG 0 
#define COMPASS_DEBUG 0
#define GPS_DEBUG 1

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
#define NAV_STATE_TAKEOFF 1
#define TAKEOFF_TIME 3000  // ms
#define NAV_STATE_CLIMB 2
#define NAV_STATE_NAVIGATE 3
#define INVALID_NAV -1000
#define ARRIVED_THRESHOLD 0.0001

// dynamics
#define CLIMB_PITCH 15
#define RECOVER_PITCH -15

// mechanical controls
#define YAW_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define YAW_MECHANICAL_MAX 25  // mechanical limits of servo travel


#endif
