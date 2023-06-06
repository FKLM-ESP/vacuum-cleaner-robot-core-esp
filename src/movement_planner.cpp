#include "movement_planner.h"

/*
    My idea (Lorenzo) is that planning and actuation will be separated, mediated by control variables.
    The analogy that comes to mind is the captain of a ship in the control room moving the lever that
    signals to the people in the engine bay how much power he wants. The sailors will then see the message
    and control the engines/shovel the right amount of coal.

    The functions in this file are the "planners", they will receive the variables necessary for their
    decision-making process, and to variables that will signal to the actuators what to do.

    The first one should be launched as a thread, as it is the automatic mode planner.

    The second one should be called with each incoming message only when the device is in auto mode.

*/


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


/*
    PROPOSAL FOR IMPLEMENTATION
    This function will be called for every WiFi message related to manual control, and will receive a
    pointer to a "current control" variable.
    It will read the incoming message and change the variable accordingly, only if such change is allowed.

    The control variable will contain values meaning "forwards", "backwards", "rotate right", "rotate left".

    This function will be called on a per-message basis by the thread/function responsible for reading
    incoming WiFi messages.
*/
void handleControls()
{
    // TODO: handle control commands sent from GUI
}