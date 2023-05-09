#include "Motor_Controller.h"

MotorController::MotorController(
    PinName l_in1, PinName l_in2,
    PinName r_in1, PinName r_in2) : 
    _left(Motor(l_in1, l_in2, false)),
    _right(Motor(r_in1, r_in2, true)) {}

void MotorController::moveForward()
{
    _left.speed(LEFT_MOTOR_SPEED);
    _right.speed(RIGHT_MOTOR_SPEED);
}

void MotorController::moveBackwards()
{
    _left.speed(-LEFT_MOTOR_SPEED);
    _right.speed(-RIGHT_MOTOR_SPEED);
}

void MotorController::rotateRight()
{
    _left.speed(LEFT_MOTOR_SPEED);
    _right.speed(-RIGHT_MOTOR_SPEED);
}

void MotorController::rotateLeft()
{
    _left.speed(-LEFT_MOTOR_SPEED);
    _right.speed(RIGHT_MOTOR_SPEED);
}

void MotorController::stop()
{
    _left.speed(0);
    _right.speed(0);
}