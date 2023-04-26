#include "mbed.h"
#include <chrono>

class Ultrasonic
{
public:
    Ultrasonic(PinName t, PinName e);

    float echo_duration();
    float distance();

private:
    DigitalOut trig;
    DigitalIn echo;
    Timer timer;
    float duration,distance_cm;
};
