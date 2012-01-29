#ifndef CONFIG_H
#define CONFIG_H

// program flow
#define WAIT_FOR_GPS_LOCK 0

// pins
#define SERVO_PIN 9
#define GPS_MUX_PIN 7
#define SOFT_SERIAL_RX 11
#define SOFT_SERIAL_TX 12

// output config
#define SERIAL_RATE 38400
#define SOFT_SERIAL_RATE 9600
#define NAV_DEBUG 1
#define CONTROLLER_DEBUG 1 
#define COMPASS_DEBUG 1
#define GPS_DEBUG 1

// real time config (millis)
#define GPS_RT_RATE 250
#define COMPASS_RT_RATE 10
#define MPU_RT_RATE 0

// Navigator controls
#define MAX_WAYPOINTS 7
#define ARRIVED_THRESHOLD 0.03048  // 100 ft in km
#define GPS_MIN_SPEED_THRESHOLD 0.8689  // 1 mph in 100ths of knots

#endif
