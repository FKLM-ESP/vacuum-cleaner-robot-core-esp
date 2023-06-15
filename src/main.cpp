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

// communication variables
ESP8266Interface wifi(PA_9, PB_3);
#define WIFI_SSID "ExtRouter"
#define WIFI_PASSWORD "easy-p@ss87"
#define PORT 9000
#define IP_ADDR "192.168.1.10" // Lorenzo phone
// #define IP_ADDR "192.168.1.11"  // Khalil phone
// #define IP_ADDR "192.168.1.12"  // Filip phone

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

// Global variables
int currentCoordsSize;
int *coords;

int *position_3d;
float *velocity_3d;
float *orientation_3d;
float *ang_velocity_3d;

control_mode current_mode;
bool fan_state;
uint8_t current_movement_state;

// general control variables
DigitalOut led1(LED1);
DigitalIn button(USER_BUTTON);

bool buttonDown = false;

void handleButton()
{
    // TODO: make this function a thread, which uses .recv to check for commands from UI
    if (button)
    { // button is pressed
        if (!buttonDown)
        { // a new button press
            if (current_mode == automatic)
            {
                current_mode = manual;
            }
            else
            {
                current_mode = automatic;
            }
            buttonDown = true;    // record that the button is now down so we don't count one press lots of times
            thread_sleep_for(10); // ignore anything for 10ms, a very basic way to de-bounce the button.
        }
    }
    else
    { // button isn't pressed
        buttonDown = false;
    }
}

int main()
{
    // Global variable allocation
    coords = (int *)malloc(sizeof(int) * MAX_COORDS);
    coords[0] = 0; coords[1] = 0;
    currentCoordsSize = 2;

    position_3d = (int *)malloc(sizeof(int) * 3);
    position_3d[0] = 0; position_3d[1] = 0; position_3d[2] = 0;
    velocity_3d = (float *)malloc(sizeof(float) * 3);
    velocity_3d[0] = 0; velocity_3d[1] = 0; velocity_3d[2] = 0;
    orientation_3d = (float *)malloc(sizeof(float) * 3);
    orientation_3d[0] = 0; orientation_3d[1] = 0; orientation_3d[2] = 0;
    ang_velocity_3d = (float *)malloc(sizeof(float) * 3);
    ang_velocity_3d[0] = 0; ang_velocity_3d[1] = 0; ang_velocity_3d[2] = 0;

    fan_state = false;
    current_movement_state = STATE_STOP;
    // ??? current_mode = auto;

    printf("This is the vacuum cleaner core running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    imu_i2c.frequency(400000);

    // run_hw_check_routine(imu, controller, sensor_1, sensor_2, &wifi);

    // Connect to Wi-Fi
    printf("\r\nConnecting...\r\n");
    SocketAddress a;
    int ret = wifi.connect(WIFI_SSID, WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0)
    {
        printf("\r\nCan't connect to wi-fi. Retrying\r\n");
        // TODO: Led indicator?
    }

    printf("Connected to WiFi!\r\n\r\n");

    TCPSocket socket;
    socket.open(&wifi);
    wifi.gethostbyname(IP_ADDR, &a); // address might be device dependent ???
    a.set_port(PORT);
    socket.connect(a);

    Timer timer;
    timer.start();

    /*
        Missing:
            - start motor actuator thread, if necessary (functionality can be handled by the main loop)
            - start imu reading thread, will be responsible from reading values and updating position_3d and orientation_3d
            - start auto mode thread, if "too cumbersome" to be handled in the main loop
                Pro thread
                    less logic in main
                    faster reaction (maybe)
                Pro no-thread
                    easier to handle, not starting and stopping threads
                    we don't really care about speed

        The loop will be responsible for
            - handling the transition between automatic and manual modes (=starting and stopping the auto mode thread if used)
                NOTE: if auto mode is not controlled by a separate thread than the 'new_mode' variable is useless, the ui command
                    reader can directly set 'current_mode' and the main loop will pick up from there
            - periodically sending data to the app, with different periods (different timers) depending on the importance and velocity
                (e.g. imu every second, coordinates every 10 seconds, battery every 20 seconds)
            - if we want to handle wifi and socket reconnection
                MUST use separate threads for this, connection is very slow
            - set motor controller state if not using a separate thread
    */

    // while (true)
    {
        // send battery level, coordinates and IMU data every 1 second
        // if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 3.0)
        {
            // sendBattery(&socket, &battery_reader);

            // sendLog(&socket, "Test");

            // sendCoordinates(&socket);

            // sendIMU(&socket, &imu);

            test_imu(imu);

            timer.reset();
        }
    }

    // current_mode = test;

    // while (true)
    // {
    //     switch (current_mode)
    //     {
    //         case test:
    //             handleButton();
    //             run_hw_check_routine(imu, controller, sensor, &wifi);
    //             current_mode = manual;
    //             break;
    //         case manual:
    //             led1 = true;
    //             handleButton();
    //             handleControls();
    //             break;
    //         case automatic:
    //             led1 = false;
    //             handleButton();
    //             autoClean();
    //             break;
    //     }

    //     // send battery level, coordinates and IMU data every 1 second
    //     if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 1.0)
    //     {
    //         sendBattery(&socket);

    //         sendCoordinates(&socket);

    //         sendIMU(&socket);

    //         timer.reset();
    //     }

    //     // TODO: what happens if Wi-Fi disconnects / TCP socket fails
    // }

    free(coords);
    free(position_3d);
    free(orientation_3d);
    free(ang_velocity_3d);
}