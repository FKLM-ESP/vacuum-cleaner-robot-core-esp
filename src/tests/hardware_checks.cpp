#include "hardware_checks.h"

int run_hw_check_routine(BMI160_I2C bmx, MotorController contr, Ultrasonic sensor1, Ultrasonic sensor2, ESP8266Interface *wifi_module, bool wifi_test, DigitalOut *led_test, DigitalOut *led_fan)
{
    printf("HW check initialized.\n");

    if (wifi_test)
    {
        test_wifi(wifi_module);
        printf("WiFi tested.\n");
    }

    if (test_imu(bmx))
    {
        *led_test = 1;
        thread_sleep_for(2000);
        *led_test = 0;
    }
    printf("IMU tested.\n");

    test_motor_controller(contr);
    printf("Motor controller tested.\n");
    
    if (test_ultrasonic(sensor1)) || test_ultrasonic(sensor2))
    {
        *led_fan = 1;
        thread_sleep_for(2000);
        *led_fan = 0;
    }
    printf("Ultrasonic tested.\n");

    return 0;
}
