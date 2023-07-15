#include "hardware_checks.h"

int runHwCheckRoutine(BMI160_I2C bmx, MotorController contr, Ultrasonic sensor1, Ultrasonic sensor2, ESP8266Interface *wifi_module, bool wifi_test, DigitalOut *led_test, DigitalOut *led_fan)
{
    printf("HW check initialized.\n");

    if (wifi_test)
    {
        testWifi(wifi_module);
        printf("WiFi tested.\n");
    }

    if (testIMU(bmx))
    {
        *led_test = 1;
        thread_sleep_for(2000);
        *led_test = 0;
    }
    printf("IMU tested.\n");

    testMotorController(contr);
    printf("Motor controller tested.\n");
    
    if (testUltrasonic(sensor1) || testUltrasonic(sensor2))
    {
        *led_fan = 1;
        thread_sleep_for(2000);
        *led_fan = 0;
    }
    printf("Ultrasonic tested.\n");

    return 0;
}
