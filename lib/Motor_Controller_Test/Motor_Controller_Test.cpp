#include "Motor_Controller_Test.h"

void test_motor_controller(MotorController contr)
{
    contr.moveForward();

    thread_sleep_for(1000);

    contr.stop();
    contr.moveBackwards();

    thread_sleep_for(1000);

    contr.stop();
    contr.rotateLeft();

    thread_sleep_for(1000);

    contr.stop();
    contr.rotateRight();

    thread_sleep_for(1000);

    contr.stop();

    return;
}