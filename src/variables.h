// map variables
// flattened tuple (x,y), updated on wall hit, stores initial and current position

// Constants

#define MAX_COORDS 1024
const int bytesPerCoord = sizeof(int);
const int bytesPerIMUValue = sizeof(float);

// Global variables

// Be careful: ADD THE COORDINATES BEFORE INCREASING currentCoordsSize,
//  or use the usual coords[currentCoordsSize++] = newCoord

// Always check that currentCoordsSize < MAX_COORDS before adding

// All files accessing these variables should re-declare them in their .h file (without extern)
extern int currentCoordsSize = 0; // increase by two with each new coordinate
extern int coords[MAX_COORDS] = {};

