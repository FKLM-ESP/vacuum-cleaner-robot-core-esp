#include "Motor_Controller.h"

MotorController::MotorController(
    PinName l_pwm, PinName l_fwd, PinName l_rev,
    PinName r_pwm, PinName r_fwd, PinName r_rev) : _left(Motor(l_pwm, l_fwd, l_rev)),
                                                   _right(Motor(r_pwm, r_fwd, r_rev)) {}

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