#include "mbed.h"
#include "variables.h"
#include "WiFi_interface.h"

/*
    Checking for obstacles on the path closer than set threshold and move forward if there are none. 
    If obstacles are detected, rotate randomly clockwise or counterclockwise in a direction.
    Once rotation is finished, continue moving forward.
*/
void planMovement();