#include "Motor_Controller_test.h"

int testMotorController(MotorController contr)
{
    contr.moveForward();
    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(1000);

    contr.moveBackwards();
    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(1000);

    contr.rotateLeft();
    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(1000);

    contr.rotateRight();
    thread_sleep_for(1000);

    contr.stop();
    thread_sleep_for(1000);

    return 0;
}