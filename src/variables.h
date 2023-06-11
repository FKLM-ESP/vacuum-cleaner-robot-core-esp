// map variables
// flattened tuple (x,y), updated on wall hit, stores initial and current position
#ifndef VARIABLES_H

#define VARIABLES_H
// Constants

#define MAX_COORDS 1024
const int bytesPerCoord = sizeof(int);
const int bytesPerIMUValue = sizeof(float);

// Global variables

// Be careful: ADD THE COORDINATES BEFORE INCREASING currentCoordsSize,
//  or use the usual coords[currentCoordsSize++] = newCoord

// Always check that currentCoordsSize < MAX_COORDS before adding

extern int currentCoordsSize; // increase by two with each new coordinate
extern int *coords;

#endif