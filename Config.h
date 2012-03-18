#ifndef CONFIG_H
#define CONFIG_H

// program flow
#define WAIT_FOR_GPS_LOCK 1
#define WAIT_FOR_SLAVE_ACK
#define TEST_LINK_DURATION 5000  // ms
#define TEST_LINK_ERROR_THRESHOLD 0.01  // 1%

#define MPU_SS_PIN 4
#define GPS_MUX_PIN 7
#define MINI_SS_PIN 10
#define AIRSPEED_PIN 3
#define BATTERY_PIN 1
#define RADIO_MUX_PIN 2
#define SPI_SLAVE_ACK_PIN 8

// sensor configs
#define GYRO_X_SIGN 1
#define GYRO_Y_SIGN 1
#define GYRO_Z_SIGN 1
#define ACCEL_X_SIGN 1
#define ACCEL_Y_SIGN -1
#define ACCEL_Z_SIGN -1
#define MAG_X_SIGN 1
#define MAG_Y_SIGN -1
#define MAG_Z_SIGN 1

// output config
#define SERIAL_RATE 38400 // baud

// debug config
#define NAV_DEBUG 0
#define PILOT_DEBUG 0
#define CONTROLLER_DEBUG 0 
#define COMPASS_DEBUG 0
#define GPS_DEBUG 0

// compass settings
#define MAG_DECLINATION -6

// analog settings
#define VREF 5.0
#define MAX_ADC_RANGE 1024

// DCM controls
#define GYRO_WEIGHT 0
#define MAG_WEIGHT 0.5
#define ACCEL_WEIGHT 0.5

// Navigator controls
#define EARTH_RADIUS 6371  // km avg earth radius
#define MAX_WAYPOINTS 7
#define HOLD_PATTERN_WAYPOINTS 8
#define HOLD_PATTERN_RADIUS 0.050  // km
#define STATE_INIT -4
#define STATE_END -3
#define STATE_GLIDE -2
#define STATE_RECOVER -1
#define STATE_START 0
#define START_DURATION 3000 // ms
#define STATE_TAKEOFF 1
#define TAKEOFF_DURATION 3000  // ms
#define STATE_CLIMB 2
#define STATE_NAVIGATE 3
#define INVALID_NAV -1
#define ARRIVED_THRESHOLD 0.02

// dynamics
#define MIN_AIR_SPEED 2
#define CRUISE_AIR_SPEED 10  // m/s ~22 mph
#define MAX_AIR_SPEED 20  // m/s ~44 mph
#define CRUISE_ALTITUDE 228  // m ~800 ft
#define CRUISE_ALTITUDE_THRESHOLD 10  // at CRUISE_ALTITUDE+/-CRUISE_ALTITUDE_THRESHOLD pitch control reaches +/- PITCH_MAX 
#define PITCH_MAX 20  // degrees
#define CLIMB_PITCH 15  // degrees
#define RECOVER_PITCH -15  // degrees
#define THROTTLE_MAX_RATE 25  // %/sec
#define THROTTLE_MIN 10  // %
#define THROTTLE_MAX 90  // %

// mechanical controls
#define PITCH_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define PITCH_MECHANICAL_MAX 25  // mechanical limits of servo travel
#define YAW_SERVO_POLARITY -1  // 1 normal polarity; -1 reverse polarity of yaw servo
#define YAW_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define YAW_MECHANICAL_MAX 25  // mechanical limits of servo travel
#define ROLL_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define ROLL_MECHANICAL_MAX 25  // mechanical limits of servo travel


#endif
