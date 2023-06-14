#include "movement_planner.h"

/*
    PROPOSAL FOR IMPLEMENTATION

    This function will be launched inside a new thread. It will receive pointers to current pose
    (6DOF), to the control variable and to the ultrasonic sensors and bumper buttons (if used).

    It will be responsible for checking for collision, setting the control to "stop", then setting
    it to rotate either clockwise or counterclockwise. Then it will sleep for a random time duration
    in an interval (defined using real-life measurements), then set it to "stop" and enter the
    "check for collision" loop again.
*/

void autoClean()
{
    // TODO: handle automatic cleaning algorithm
}
