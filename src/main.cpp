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
#include "imu_reader.h"
#include "movement_planner.h"

// communication variables
ESP8266Interface wifi(PA_9, PB_3);
TCPSocket socket;

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
    current_mode = manual;
    new_mode = manual;

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
    if(imu.startFastCalibration()) {
        printf("Failed to start fast calibration.\n");
    }

    // run_hw_check_routine(imu, controller, sensor_1, sensor_2, &wifi, true, &led_test, &led_fan);

    if (0 == connect_to_wifi())
    {
        connect_to_socket();
    }

    /***********************************************************************************
     * TEST AREA
     ***********************************************************************************/

    // Timer timer;
    // timer.start();

    // while (true)
    // {
    //     // send battery level, coordinates and IMU data every 1 second
    //     if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 0.5)
    //     {
    //         sendBattery(&socket, &battery_reader);

    //         // sendLog(&socket, "Test");

    //         sendCoordinates(&socket);

    //         sendIMU(&socket, &imu);

    //         timer.reset();
            
    //     }
    // }

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

#if 1
    Timer timerImu, timerCoordinates, timerBatteries;
    timerImu.start();
    timerCoordinates.start();
    timerBatteries.start();

    // // not reused, can be a variable
    Thread imu_reader_thread; 
    // // reused, need to be pointers that are allocated every time
    Thread *auto_mode_thread = NULL;
    Thread *wifi_connector_thread = NULL;
    //Thread *socket_connector_thread = NULL;

    // Start imu reading thread
    imu_reader_thread.start(callback(imu_read_and_update_coords, &imu));

    // Start auto mode thread
    // auto_mode_thread = new Thread;
    // auto_mode_thread->start(autoClean);

    sendLog(&socket, "Starting main loop");

    while (true)
    {
        readCommand(&socket);
        //handle mode changes first
        if (current_mode != new_mode)
        {
            if (current_mode == automatic)
            {
                // Stop auto thread
                auto_mode_thread->terminate();
                delete auto_mode_thread;
                auto_mode_thread = NULL;

                // Set new_movement_state and current_movement_state to STATE_STOP
                new_movement_state = STATE_STOP;

                // Turn off the fan
                led_fan = 0;

                sendLog(&socket, "Exited automatic mode, stopped auto thread and fan");
            }

            if (new_mode == automatic)
            {
                // Set new_movement_state and current_movement_state to STATE_STOP
                new_movement_state = STATE_STOP;

                // Start auto thread
                auto_mode_thread = new Thread;
                auto_mode_thread->start(autoClean);

                // Turn on the fan
                led_fan = 1;

                sendLog(&socket, "Changed mode to automatic, started auto thread and fan");
            }
            else if (new_mode == manual)
            {
                sendLog(&socket, "Changed mode to manual");
            } 
            else
            {
                sendLog(&socket, "Changed mode to test");
            }
            
            // Set current_mode to new_mode
            current_mode = new_mode;
        }

        switch (current_mode)
        {
        case test:
            run_hw_check_routine(imu, controller, sensor_1, sensor_2, &wifi, false, &led_test, &led_fan);
            new_mode = manual;
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

        // Check for wifi connection, if not connected and thread not running start it
        if (wifi_connector_thread != NULL &&
            (wifi_connector_thread->get_state() == rtos::Thread::State::Deleted ||
            wifi_connector_thread->get_state() == rtos::Thread::State::Inactive) )
        {
            wifi_connector_thread->terminate();
            delete wifi_connector_thread;
            wifi_connector_thread = NULL;
        }

        // Reuse battery timer to avoid continuous connection attempts
        if (std::chrono::duration<float>{timerBatteries.elapsed_time()}.count() >= 20.0 &&
            wifi_connector_thread == NULL &&
            (wifi.get_connection_status() < 0 ||
            wifi.get_connection_status() == NSAPI_STATUS_DISCONNECTED) )
        {
            wifi_connector_thread = new Thread;
            wifi_connector_thread->start(connect_to_wifi);
        }

        // // Check for socket connection, if not connected and thread not running start it
        // if (socket_connector_thread != NULL &&
        //     (socket_connector_thread->get_state() == rtos::Thread::State::Deleted ||
        //     socket_connector_thread->get_state() == rtos::Thread::State::Inactive) )
        // {
        //     socket_connector_thread->terminate();
        //     delete socket_connector_thread;
        //     socket_connector_thread = NULL;
        // }

        // // Reuse battery timer to avoid continuous connection attempts
        // if (std::chrono::duration<float>{timerBatteries.elapsed_time()}.count() >= 20.0 &&
        //     socket_connector_thread == NULL &&
        //     wifi.get_connection_status() >= 0 &&
        //     wifi.get_connection_status() != NSAPI_STATUS_DISCONNECTED)
        // {
        //     socket_connector_thread = new Thread;
        //     socket_connector_thread->start(connect_to_socket);
        // }

        // check for timers expiration and send messages
        if (std::chrono::duration<float>{timerImu.elapsed_time()}.count() >= 1.0)
        {
            sendIMU(&socket, &imu);
            printf("X: %d\tY: %d\tZ: %d\tVel_x: %2.4f\tVel_y: %2.4f\tVel_z: %2.4f\tYaw: %2.4f\tPitch: %2.4f\tRoll: %2.4f\n", POS_X, POS_Y, POS_Z, VEL_X, VEL_Y, VEL_Z, YAW, PITCH, ROLL);
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