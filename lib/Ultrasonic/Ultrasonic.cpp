#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(DigitalOut t, DigitalIn e)
: trig(t), echo(e){}

float Ultrasonic::echo_duration()
{
    timer.reset();  //reset timer

    trig.write(0);   // trigger low 
    wait_us(2); //  wait 

    trig.write(1);   //  trigger high
    wait_us(10);
    trig.write(0);  // trigger low

    while(!echo); // start pulseIN
    timer.start();

    while(echo);
    timer.stop();

    return std::chrono::duration<float>{timer.elapsed_time()}.count();
}

//return distance in cm 
float Ultrasonic::distance() {
    duration = echo_duration();
    distance_cm = duration * 34000 / 2; 
    return distance_cm;

}