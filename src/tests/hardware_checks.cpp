#include "hardware_checks.h"

DigitalOut led_pcb(PA_10);

int run_hw_check_routine(BMI160_I2C bmx, MotorController contr, Ultrasonic sensor1, Ultrasonic sensor2, ESP8266Interface *wifi_module)
{
    printf("HW check initialized.\n");

    //test_motor_controller(contr);

    //TESTED components below
    test_wifi(wifi_module);
    if (test_ultrasonic(sensor1) || test_ultrasonic(sensor2))
    {
        led_pcb = 1;
    }
    test_imu(bmx);
    return 0;
}
