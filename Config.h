#ifndef CONFIG_H
#define CONFIG_H

// program flow
#define WAIT_FOR_GPS_LOCK 1
#define WAIT_FOR_SLAVE_ACK
#define TEST_LINK_DURATION 5000  // ms
#define TEST_LINK_ERROR_THRESHOLD 0.01  // 1%

// pins
#define MPU_SS_PIN 4
#define GPS_MUX_PIN 7
#define MINI_SS_PIN 10
#define AIRSPEED_PIN 3
#define AIRSPEED_PIND A3
#define BATTERY_PIN 1
#define RADIO_MUX_PIN 2
#define SPI_SLAVE_ACK_PIN 8
#define RED_LED 5
#define BLUE_LED 6

// sensor configs
#define GYRO_X_SIGN -1
#define GYRO_Y_SIGN 1
#define GYRO_Z_SIGN -1
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

// sensor controls
#define CALIBRATION_ROUNDS 5
#define MAG_DECLINATION -6
#define VREF50 5.0f
#define VREF33 3.3f

// analog settings
#define VREF 5.0
#define MAX_ADC_RANGE 1024

// DCM controls
#define GYRO_WEIGHT 0.5
#define MAG_WEIGHT 0.25
#define ACCEL_WEIGHT 0.25

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
#define MIN_AIR_SPEED 5.14  // m/s ~11mph
#define CRUISE_AIR_SPEED 12.86  // m/s ~28.7kts
#define MAX_AIR_SPEED 22.35  // m/s ~50 mph

#define CRUISE_ALTITUDE 228.0  // m ~800 ft
#define CONTROL_ALTITUDE_THRESHOLD 10.0  // at CRUISE_ALTITUDE+/-CRUISE_ALTITUDE_THRESHOLD pitch control reaches +/- PITCH_MAX 
#define MAX_PITCH_ANGLE 20.0  // degrees
#define MAX_PITCH_RATE 5.0  // degrees/second
#define CLIMB_PITCH 15.0  // degrees
#define RECOVER_PITCH -15.0  // degrees
#define MAX_ELEVATOR_RATE 60.0  // degrees/second

#define THROTTLE_MAX_RATE 25.0  // %/sec
#define THROTTLE_MIN 10.0  // %
#define THROTTLE_MAX 90.0  // %

// mechanical controls
#define ELEVATOR_SERVO_POLARITY -1.0  //  1 normal polarity; -1 reverse polarity of elevator servo
#define ELEVATOR_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define ELEVATOR_MECHANICAL_MAX 25  // mechanical limits of servo travel
#define RUDDER_SERVO_POLARITY -1.0  // 1 normal polarity; -1 reverse polarity of yaw servo
#define RUDDER_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define RUDDER_MECHANICAL_MAX 25  // mechanical limits of servo travel
#define AILERON_CENTER_ANGLE 90  // approximate steering on-center angle for servo
#define AILERON_MECHANICAL_MAX 25  // mechanical limits of servo travel

#endif
