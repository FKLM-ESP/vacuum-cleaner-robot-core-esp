#include "hardware_checks.h"

int run_hw_check_routine(IMU_BMX160 bmx, MotorController contr, Ultrasonic sensor, ESP8266Interface *wifi_module)
{
    printf("HW check initialized.\n");

    //test_motor_controller(contr);
    //test_imu(bmx);
    //test_ultrasonic(sensor);
    test_wifi(wifi_module);
    return 0;
}
