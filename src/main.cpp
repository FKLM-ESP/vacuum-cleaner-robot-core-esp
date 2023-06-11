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
#define IP_ADDR "192.168.1.10"  // Lorenzo phone
//#define IP_ADDR "192.168.1.11"  // Khalil phone
//#define IP_ADDR "192.168.1.12"  // Filip phone

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

// Coordinates 
int currentCoordsSize;
int *coords;

// general control variables
DigitalOut led1(LED1);
DigitalIn button(USER_BUTTON);
enum control_mode
{
    automatic,
    manual,
    test
} mode;
bool buttonDown = false;

void handleButton()
{
    // TODO: make this function a thread, which uses .recv to check for commands from UI
    if (button)
    { // button is pressed
        if (!buttonDown)
        { // a new button press
            if (mode == automatic)
            {
                mode = manual;
            }
            else
            {
                mode = automatic;
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
    // Initialization
    coords = (int*)malloc(sizeof(int) * MAX_COORDS);
    currentCoordsSize = 0;

    printf("This is the vacuum cleaner core running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
    
    imu_i2c.frequency(400000);
    
    //run_hw_check_routine(imu, controller, sensor_1, sensor_2, &wifi);

    // Connect to Wi-Fi
    printf("\r\nConnecting...\r\n");
    SocketAddress a;
    int ret = wifi.connect(WIFI_SSID, WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0)
    {
        printf("\r\nCan't connect to wi-fi. Retrying\r\n");
        //TODO: Led indicator?
    }

    printf("Connected to WiFi!\r\n\r\n");

    TCPSocket socket;
    socket.open(&wifi);
    wifi.gethostbyname(IP_ADDR, &a); // address might be device dependent ???
    a.set_port(PORT);
    socket.connect(a);

    Timer timer;
    timer.start();

    while (true)
    {
        // send battery level, coordinates and IMU data every 1 second
        if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 1.0)
        {
            sendBattery(&socket, &battery_reader);

            sendLog(&socket, "Test");

            // sendCoordinates(&socket);

            // sendIMU(&socket);

            timer.reset();
        }
    }


    // mode = test;

    // while (true)
    // {
    //     switch (mode)
    //     {
    //         case test:
    //             handleButton();
    //             run_hw_check_routine(imu, controller, sensor, &wifi);
    //             mode = manual;
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
}