#ifndef CUSTOM_ULTRASONIC_H
#define CUSTOM_ULTRASONIC_H

#include "mbed.h"
#include <chrono>

class Ultrasonic
{
public:
    Ultrasonic(DigitalOut t, DigitalIn e);

    float echo_duration();
    float distance();

private:
    // Note: Causes warnings about deprecation, probably should fix that at some point
    DigitalOut trig;
    DigitalIn echo;
    Timer timer;
    float duration,distance_cm;
};

#endif