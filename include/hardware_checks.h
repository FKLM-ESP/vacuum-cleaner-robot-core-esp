#include "IMU_test.h"
#include "Motor_Controller_test.h"
#include "Ultrasonic_test.h"
#include "WiFi_test.h"

int run_hw_check_routine(IMU_BMX160 bmx, MotorController contr, Ultrasonic sensor, ESP8266Interface *wifi_module);