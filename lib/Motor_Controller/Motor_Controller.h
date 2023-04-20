#include "mbed.h"
#include "Motor.h"

#define LEFT_MOTOR_SPEED 0.5
#define RIGHT_MOTOR_SPEED 0.5

class MotorController
{
public:
    MotorController(
        PinName l_pwm, PinName l_fwd, PinName l_rev,
        PinName r_pwm, PinName r_fwd, PinName r_rev);

    void moveForward();
    void moveBackwards();
    void rotateRight();
    void rotateLeft();
    void stop();

private:
    Motor _left;
    Motor _right;
};
