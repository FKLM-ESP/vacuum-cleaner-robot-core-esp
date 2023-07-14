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
    static bool handling_obstacle = false; // TODO: are we sure about this?
    float right_sensor;
    float left_sensor;
    

    if (!handling_obstacle)
    {
        new_movement_state = STATE_FORWARD;

        right_sensor = sensor_1.distance();
        thread_sleep_for(50);
        left_sensor = sensor_2.distance();

        // collision "imminent"
        if (right_sensor < DISTANCE_SENSOR_THRESH 
            || left_sensor < DISTANCE_SENSOR_THRESH)
        {
            //sendLog(&socket, "CurrentCoordSize = " + std::to_string(currentCoordsSize));
            //sendLog(&socket, "Detected obstacle, STOPping");
            printf("Detected obstacle, Stopping\n");

            // Save current coordinates to memory
            if (currentCoordsSize + 2 <= 128)
            {
                coords[currentCoordsSize++] = 95; // POS_X;
                coords[currentCoordsSize++] = 95; // POS_Y;
            }
            //sendLog(&socket, "Saved position");

            // Randomly determine new target heading, between approx. [85, 275] deg
            float rotation_amount = 1.5 + (rand() / (RAND_MAX / (4.8 - 1.5)));
            //sendLog(&socket, "Selected rotation amount");
            target_yaw = YAW + rotation_amount;
            if (target_yaw > 6.28319)
            {
                target_yaw -= 6.28319;
            }
            else if (target_yaw < 0)
            {
                target_yaw += 6.28319;
            }
            //sendLog(&socket, "Rotating");
            //sendLog(&socket, "Rotating by " + std::to_string(rotation_amount) + " radians counter-clockwise");
            printf("Rotating by %2.6f radians counter-clockwise", rotation_amount);

            // Yaw is counter-clockwise, so we select rotation direction accordingly
            new_movement_state = (rotation_amount < 3.14159) ? 
                                        STATE_LEFT : 
                                        STATE_RIGHT;

            handling_obstacle = true;
        }
    }
    else
    {
        // Wait until YAW is closer than +- YAW_TARGET_THRASH to target_yaw
        if (target_yaw - YAW_TARGET_THRESH < YAW &&
                target_yaw + YAW_TARGET_THRESH > YAW) 
        {
            //sendLog(&socket, "Reached target heading, STOPping rotation, moving FORWARD");
            printf("Reached target heading, STOPping rotation, moving FORWARD");
            new_movement_state = STATE_STOP;
            handling_obstacle = false;
        }

    }
}
