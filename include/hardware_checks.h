#include "IMU_test.h"
#include "Motor_Controller_test.h"
#include "variables.h"
#include "Ultrasonic_test.h"
#include "WiFi_test.h"

int run_hw_check_routine(BMI160_I2C bmx, MotorController contr, Ultrasonic sensor1, Ultrasonic sensor2, ESP8266Interface *wifi_module, bool wifi_test, DigitalOut *led_test, DigitalOut *led_fan);