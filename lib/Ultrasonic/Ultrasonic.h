#ifndef CUSTOM_ULTRASONIC_H
#define CUSTOM_ULTRASONIC_H

#include "mbed.h"
#include <chrono>

class Ultrasonic
{
public:
    Ultrasonic(DigitalOut t, DigitalIn e);

    // use sensor
    float echo_duration();

    // calculate distance based on duration
    float distance();

private:
    // Note: Causes warnings about deprecation
    DigitalOut trig;
    DigitalIn echo;
    Timer timer;
    float duration,distance_cm;
};

#endif