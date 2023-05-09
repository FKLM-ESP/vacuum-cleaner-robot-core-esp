#ifndef CUSTOM_ULTRASONIC_H
#define CUSTOM_ULTRASONIC_H

#include "mbed.h"
#include <chrono>

class Ultrasonic
{
public:
    Ultrasonic(PinName t, PinName e);

    float echo_duration();
    float distance();

private:
    // Note: DigitalOut/In objects are instantiated as pointers, 
    // as pin names should usually be known at compile time, but here they are not
    DigitalOut* trig;
    DigitalIn* echo;
    Timer timer;
    float duration,distance_cm;
};

#endif