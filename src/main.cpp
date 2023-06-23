// native c++ includes
#include <chrono>

// mbed includes
#include "mbed.h"
#include "TCPSocket.h"
#include "ESP8266Interface.h"

// our libraries
#include <bmx160.h>
#include <Ultrasonic.h>
#include <Motor_Controller.h>
#include <WiFi_interface.h>

// our includes
#include "hardware_checks.h"
#include "wifi_and_socket_connection.h"

// communication variables
ESP8266Interface wifi(PA_9, PB_3);

// IMU
I2C imu_i2c(PB_9, PB_8);
BMI160_I2C imu(imu_i2c, BMI160_I2C::I2C_ADRS_SDO_LO);

// Ultrasonic
DigitalOut trig_1(PC_2);
DigitalIn echo_1(PC_3);
Ultrasonic sensor_1(trig_1, echo_1);

DigitalOut trig_2(PC_11);
DigitalIn echo_2(PC_10);
Ultrasonic sensor_2(trig_2, echo_2);

// Motor controller
MotorController controller(PA_1, PA_0, PB_14, PB_15);
AnalogIn not_connected(PB_0); // PCB issue

// battery monitor
AnalogIn battery_reader(PB_1);

// LEDs
DigitalOut led_test(LED2);
DigitalOut led_fan(PA_10);

// Global variables
int currentCoordsSize;
int *coords;

int *position_3d;
float *velocity_3d;
float *orientation_3d;

control_mode current_mode;
control_mode new_mode;
bool fan_state;
uint8_t current_movement_state;
uint8_t new_movement_state;

int main()
{
    // Global variable allocation
    coords = (int *)malloc(sizeof(int) * MAX_COORDS);
    coords[0] = 0;
    coords[1] = 0;
    currentCoordsSize = 2;

    position_3d = (int *)malloc(sizeof(int) * 3);
    position_3d[0] = 0;
    position_3d[1] = 0;
    position_3d[2] = 0;
    velocity_3d = (float *)malloc(sizeof(float) * 3);
    velocity_3d[0] = 0;
    velocity_3d[1] = 0;
    velocity_3d[2] = 0;
    orientation_3d = (float *)malloc(sizeof(float) * 3);
    orientation_3d[0] = 0;
    orientation_3d[1] = 0;
    orientation_3d[2] = 0;

    fan_state = false;
    current_movement_state = STATE_STOP;
    new_movement_state = STATE_STOP;
    current_mode = automatic;
    new_mode = automatic;

    printf("This is the vacuum cleaner core running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    // Setup
    imu_i2c.frequency(400000);
    if(imu.setSensorPowerMode(BMI160::GYRO, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        printf("Failed to set gyroscope power mode\n");
    }
    thread_sleep_for(100);
    if(imu.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        printf("Failed to set accelerometer power mode\n");
    }
    thread_sleep_for(100);
    if(imu.setSensorPowerMode(BMI160::MAG, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        printf("Failed to set magnetometer power mode\n");
    }
    imu.setMagnConf(); //initialize magnetometer for regular preset.
    thread_sleep_for(100);

    // run_hw_check_routine(imu, controller, sensor_1, sensor_2, &wifi, true, &led_test, &led_fan);

    TCPSocket socket;

    if (0 == connect_to_wifi(&wifi))
    {
        connect_to_socket(&wifi, &socket);
    }

    /***********************************************************************************
     * TEST AREA
     ***********************************************************************************/

    Timer timer;
    timer.start();

    while (true)
    {
        // send battery level, coordinates and IMU data every 1 second
        if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 0.5)
        {
            sendBattery(&socket, &battery_reader);

            // sendLog(&socket, "Test");

            sendCoordinates(&socket);

            sendIMU(&socket, &imu);

            timer.reset();
            
        }
    }

    /***********************************************************************************
     * END OF TEST AREA
     * START OF IMPLEMENTATION AREA
     ***********************************************************************************/

    /*
        Missing:
            - start imu reading thread, will be responsible from reading values and updating position_3d and orientation_3d
            - start auto mode thread, if "too cumbersome" to be handled in the main loop
                Pro thread
                    less logic in main
                    faster reaction (maybe)
                Pro no-thread
                    easier to handle, not starting and stopping threads
                    we don't really care about speed

        The loop will be responsible for
            - handling the transition between automatic and manual modes (=starting and stopping the auto mode thread)
            - periodically sending data to the app, with different periods (different timers) depending on the importance and velocity
                (e.g. imu every second, coordinates every 10 seconds, battery every 20 seconds)
            - if we want to handle wifi and socket reconnection
                MUST use separate threads for this, connection is very slow
            - set motor controller state
    */

#if 0
    Timer timerImu, timerCoordinates, timerBatteries;
    timerImu.start();
    timerCoordinates.start();
    timerBatteries.start();

    // TODO: start imu reading thread

    // TODO: start auto mode thread

    while (true)
    {
        readCommand(&socket);

        // handle mode changes first
        if (current_mode != new_mode)
        {
            if (current_mode == automatic)
            {
                // TODO: Stop auto thread,
                //    set current_mode to new_mode,
                //    set new_movement_state and current_movement_state to STATE_STOP

                led_fan = 0;
            }

            if (new_mode == automatic)
            {
                // TODO: set new_movement_state and current_movement_state to STATE_STOP,
                //    set current_mode to automatic,
                //    start auto thread

                led_fan = 1;
            }
        }

        switch (current_mode)
        {
        case test:
            run_hw_check_routine(imu, controller, sensor_1, sensor_2, &wifi, false, &led_test, &led_fan);
            current_mode = manual;
            break;

        case manual:
            led_fan = (fan_state) ? 1 : 0;
            // fallthrough
        case automatic:
            if (current_movement_state != new_movement_state)
            {
                // not a case because of int flags already appearing according to the compiler
                if (new_movement_state == STATE_STOP)
                    controller.stop();
                if (new_movement_state == STATE_FORWARD)
                    controller.moveForward();
                if (new_movement_state == STATE_BACKWARD)
                    controller.moveBackwards();
                if (new_movement_state == STATE_LEFT)
                    controller.rotateLeft();
                if (new_movement_state == STATE_RIGHT)
                    controller.rotateRight();

                current_movement_state = new_movement_state;
            }
            break;
        }

        // OPTIONAL: check for wifi and socket connection status
        //     if not connected, launch reconnection in new threads

        // check for timers expiration and send messages
        // could add check for active socket, but cannot find how (?)
        if (std::chrono::duration<float>{timerImu.elapsed_time()}.count() >= 1.0)
        {
            sendIMU(&socket, &imu);
            timerImu.reset();
        }
        if (std::chrono::duration<float>{timerCoordinates.elapsed_time()}.count() >= 10.0)
        {
            sendCoordinates(&socket);
            timerCoordinates.reset();
        }
        if (std::chrono::duration<float>{timerBatteries.elapsed_time()}.count() >= 20.0)
        {
            sendBattery(&socket, &battery_reader);
            timerBatteries.reset();
        }
    }
#endif // main code

    free(coords);
    free(position_3d);
    free(orientation_3d);
}