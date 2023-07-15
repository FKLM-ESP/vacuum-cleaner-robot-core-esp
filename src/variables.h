#ifndef VARIABLES_H
#define VARIABLES_H

#include "mbed.h"
#include <types.h>
#include <../lib/Ultrasonic/Ultrasonic.h>
#include "ESP8266Interface.h"
#include "TCPSocket.h"

// CONSTANTS
#define MAX_COORDS 128
const int bytesPerCoord = sizeof(int);
const int bytesPerIMUValue = sizeof(float);

/* Connection and debuggin */
#define WIFI_SSID "ExtRouter"
#define WIFI_PASSWORD "easy-p@ss87"
#define PORT 9000
//#define IP_ADDR "192.168.1.10" // Lorenzo phone
// #define IP_ADDR "192.168.1.11"  // Khalil phone
#define IP_ADDR "192.168.1.12"  // Filip phone

// Scaling value for float to int conversion (UI IMU visualization)
#define SCALING 10000
#define GRAVITY 9.81 
#define GRAVITY_MULTIPLIER 9.81 / 2
#define PI 3.14159265358979323846

// In rad - equivalent to around 2.85 deg
#define YAW_TARGET_THRESH 0.5  // 0.05
// In cm
#define DISTANCE_SENSOR_THRESH 15.0

/* Control constants
    Note that uint8_t and hex values can be compared directly
*/
#define STATE_STOP 0x00     // 0000 0000
#define STATE_FORWARD 0x01  // 0000 0001
#define STATE_BACKWARD 0x02 // 0000 0010
#define STATE_LEFT 0x04     // 0000 0100
#define STATE_RIGHT 0x08    // 0000 1000

/* Other constants
    Bits are meant numbered 1-8 from the right (smallest one)
    Bit 6 means setting auto mode
    Bit 5 means to use auto mode or not
    Bit 8 means setting the state of the fan
    Bit 7 means the new state of the fan
*/
#define AUTO_ON 0x30  // 0011 0000 (0x20 & 0x10 but switch complains)
#define AUTO_OFF 0x20 // 0010 0000
#define FAN_ON 0xC0   // 1100 0000 (0x80 & 0x40 but switch complains)
#define FAN_OFF 0x80  // 1000 0000

#define TEST_MODE 0xFF // 1111 1111

// GLOBAL VARIABLES

// tracks whether socket is connected to external device
extern bool is_connected;

/* Coordinates and position
    Be careful: ADD THE COORDINATES BEFORE INCREASING currentCoordsSize,
     or use the usual coords[currentCoordsSize++] = newCoord
    Always check that currentCoordsSize < MAX_COORDS before adding
*/

extern int currentCoordsSize; // increase by two with each new coordinate
extern int *coords;

// in m
extern int *position_3d;
// in m/s
extern float *velocity_3d;
// in radians?
extern float *orientation_3d;

extern control_mode new_mode; // controlled from the ui app to signal to the main loop to change current mode
extern control_mode current_mode;
extern float target_yaw;
extern bool fan_state;
extern uint8_t current_movement_state;
extern uint8_t new_movement_state;

// UTIL DEFINES
#define POS_X   position_3d[0]
#define POS_Y   position_3d[1]
#define POS_Z   position_3d[2]

#define VEL_X   velocity_3d[0]
#define VEL_Y   velocity_3d[1]
#define VEL_Z   velocity_3d[2]

#define YAW     orientation_3d[0]
#define PITCH   orientation_3d[1]
#define ROLL    orientation_3d[2]

extern Ultrasonic sensor_1;
extern Ultrasonic sensor_2;
extern TCPSocket socket;
extern ESP8266Interface wifi;

#endif