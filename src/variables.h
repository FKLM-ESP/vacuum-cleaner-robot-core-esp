// map variables
// flattened tuple (x,y), updated on wall hit, stores initial and current position
#ifndef VARIABLES_H
#define VARIABLES_H

#include <types.h>

// CONSTANTS
#define MAX_COORDS 1024
const int bytesPerCoord = sizeof(int);
const int bytesPerIMUValue = sizeof(float);

/* Control constants
    Note that uint8_t and hex values can be compared directly
    TODO: change the message sent by the UI app
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

// GLOBAL VARIABLES

/* Coordinates and position
    Be careful: ADD THE COORDINATES BEFORE INCREASING currentCoordsSize,
     or use the usual coords[currentCoordsSize++] = newCoo
    Always check that currentCoordsSize < MAX_COORDS before adding
*/

extern int currentCoordsSize; // increase by two with each new coordinate
extern int *coords;

// in cm
extern int *position_3d;
// in radians?
extern float *orientation_3d;

extern control_mode new_mode; // controlled from the ui app to signal to the main loop to chenge current mode
extern control_mode current_mode;
extern bool fan_state;
extern uint8_t current_movement_state;

#endif