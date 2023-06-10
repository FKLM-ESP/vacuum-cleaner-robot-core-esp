// map variables
// flattened tuple (x,y), updated on wall hit, stores initial and current position

// Constants

#define MAX_COORDS 1024
const int bytesPerCoord = sizeof(int);
const int bytesPerIMUValue = sizeof(float);

// Global variables

int currentCoordsSize = 0; // increase by two with each new coordinate
int coords[MAX_COORDS] = {};

