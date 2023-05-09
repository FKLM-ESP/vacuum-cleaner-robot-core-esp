#include "Motor.h"

Motor::Motor(PinName in1, PinName in2, bool isNCHTimer):
        _in1(in1), _in2(in2), _isNCHTimer(isNCHTimer) {

    // Set initial condition of PWM
    _in1.period(0.001);
    _in2.period(0.001);

    // Initial condition of output enables
    _in1 = 0;
    _in2 = 0;
}

void Motor::speed(float speed) {
    // If both in1 and in2 are high, the motor controller gradually slows down the motor (breaks)
    // The documentation suggests to switch between driving and breaking when using PWM
    // in1 = high, in2 = low -> Forward
    // in1 = low, in2 = high -> Reverse

    if (!_isNCHTimer) {
        // To "switch" between driving and breaking the high pin needs to have 100% duty-cycle,
        // while the low pin should be kept low at the chosen duty-cycle (speed)
        if (speed > 0.0) {
            _in1 = 1; // 100% duty-cycle 
            _in2 = 1 - abs(speed); // duty-cycle should determine low signal
        }
        else if (speed < 0.0) {
            _in1 = 1 - abs(speed); // duty-cycle should determine low signal
            _in2 = 1; // 100% duty-cycle 
        }
        else {
            // if speed = 0, break
            _in1 = 1;
            _in2 = 1;
        }
    }
    else {
        // if N-Channel of timer is used, duty cycle determines how much of period is LOW, instead of HIGH -> 0% duty cycle = digital HIGH
        // To "switch" between driving and breaking the high pin needs to have 0% duty-cycle,
        // while the low pin should be kept low at the chosen duty-cycle (speed)
        if (speed > 0.0) {
            _in1 = 0; // 0% duty-cycle -> always high
            _in2 = abs(speed); // duty-cycle already determines low signal
        }
        else if (speed < 0.0) {
            _in1 = abs(speed); // duty-cycle already determines low signal
            _in2 = 0; // 0% duty-cycle 
        }
        else {
            // if speed = 0, break
            _in1 = 0;
            _in2 = 0;
        }
    }
}
