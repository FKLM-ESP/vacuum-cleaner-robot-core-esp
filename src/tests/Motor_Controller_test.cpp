#include "Motor_Controller_test.h"

int test_motor_controller(MotorController contr)
{
    contr.moveForward();

    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(50);

    contr.moveBackwards();

    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(50);

    contr.rotateLeft();

    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(50);

    contr.rotateRight();

    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(50);

    return 0;
}