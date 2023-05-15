#include "Ultrasonic_test.h"

int test_ultrasonic(Ultrasonic sensor)
{
    for(int i = 0; i < 5; i++)
    {    
        float distance = sensor.distance();   
        printf("distance: %f \n",distance);
        wait_us(1000000); // 1 sec  
    }
    
    return 0;
}