#ifndef CUSTOM_MOTOR_CONT_H
#define CUSTOM_MOTOR_CONT_H

#include "mbed.h"
#include "Motor.h"

#define LEFT_MOTOR_SPEED 0.5
#define RIGHT_MOTOR_SPEED 0.5

class MotorController
{
public:
    MotorController(
        PinName l_in1, PinName l_in2,
        PinName r_in1, PinName r_in2);

    void moveForward();
    void moveBackwards();
    void rotateRight();
    void rotateLeft();
    void stop();

private:
    Motor _left;
    Motor _right;
};

#endif