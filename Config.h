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
#define INVALID_NAV_IDX -1
#define USE_CURLOC -1000
#define ARRIVED_THRESHOLD 0  // 100 ft in km
#define GPS_MIN_SPEED_THRESHOLD 0.8689  // 1 mph in 100ths of knots

// mechanical controls
#define YAW_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define YAW_MECHANICAL_MAX 25  // mechanical limits of servo travel


#endif
