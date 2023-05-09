// native c++ includes
#include <chrono>

// mbed includes
#include "mbed.h"
#include "TCPSocket.h"
#include "ESP8266Interface.h"

// our libraries
#include <IMU_BMX160.h>
#include <Ultrasonic.h>
#include <Motor_Controller.h>

// our includes
#include "hardware_checks.h"

// communication variables
ESP8266Interface wifi(PC_6, PC_7);
#define WIFI_SSID "ssid"
#define WIFI_PASSWORD "pass"
#define PORT 9000

// IMU
I2C imu_i2c(I2C_SDA, I2C_SCL);
IMU_BMX160 imu(&imu_i2c);

// Ultrasonic
Ultrasonic sensor(PC_2, PC_3);

// Motor controller
MotorController controller(PA_1, PB_10, PB_14, PB_15);

// map variables
// flattened tuple (x,y), updated on wall hit, stores initial and current position
#define MAX_COORDS 1024
int currentCoordsSize = 0; // increase by two with each new coordinate
int coords[MAX_COORDS] = {};

// general control variables
DigitalOut led1(LED1);
DigitalIn button(USER_BUTTON);
enum control_mode {automatic, manual, test} mode;
bool buttonDown = false;


uint8_t readBattery()
{
    //TODO: read battery level
    return 100;
}

void autoClean()
{
    //TODO: handle automatic cleaning algorithm
}

void handleControls()
{
    //TODO: handle control commands sent from GUI
}

void handleButton()
{
    //TODO: make this function a thread, which uses .recv to check for commands from UI
    if (button) 
    {  // button is pressed
        if  (!buttonDown) 
        {  // a new button press
            if (mode == automatic) 
            {
                mode = manual;
            }
            else 
            {
                mode = automatic;
            }
            buttonDown = true;     // record that the button is now down so we don't count one press lots of times
            thread_sleep_for(10);  // ignore anything for 10ms, a very basic way to de-bounce the button. 
        } 
    } else 
    { // button isn't pressed
        buttonDown = false;
    }

}

int main()
{
    printf("This is the vacuum cleaner core running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    // Connect to Wi-Fi
    SocketAddress a;
    while (wifi.connect(WIFI_SSID, WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2) != 0)
    {
        printf("\r\nCan't connect to wi-fi. Retrying\r\n");
    }

    printf("Success\r\n\r\n");
    printf("MAC: %s\r\n", wifi.get_mac_address());
    wifi.get_ip_address(&a);
    printf("IP: %s\r\n", a.get_ip_address());
    wifi.get_netmask(&a);
    printf("Netmask: %s\r\n", a.get_ip_address());
    wifi.get_gateway(&a);
    printf("Gateway: %s\r\n", a.get_ip_address());
    printf("RSSI: %d\r\n\r\n", wifi.get_rssi());

    TCPSocket socket;
    socket.open(&wifi);
    wifi.gethostbyname("localhost", &a);
    a.set_port(PORT);
    socket.connect(a);

    Timer timer;
    timer.start();

    mode = test;

    while (true)
    {
        switch(mode)
        {
            case test:
                handleButton();
                run_hw_check_routine(imu, controller, sensor, &wifi);
                break;
            case manual:
                led1 = true;
                handleButton();
                handleControls();
                break;
            case automatic:
                led1 = false;
                handleButton();
                autoClean();
                break;
        }

        // send battery level, coordinates and IMU data every 1 second
        if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 1.0)
        {
            uint8_t batLvl = readBattery();

            // send battery level
            char batMsg[10];
            std::sprintf(batMsg,"b%hhu", batLvl);
            socket.send(batMsg, sizeof batMsg);

            // send coordinates
            char coordMsg[MAX_COORDS];
            coordMsg[0] = 'c';
            for (int i = 0; i < MAX_COORDS; i++)
            {
                if (i == currentCoordsSize) break;
                
                // Note: values above 255 won't work!
                coordMsg[i + 1] = char(coords[i]); // needs to be converted to int in UI

            }
            
            // TODO: test below line
            socket.send(coordMsg, sizeof(char) * currentCoordsSize);

            // send IMU data
            sBmx160SensorData_t mag, gyr, acc;
            imu.getAllData(&mag, &gyr, &acc);
            char imuMsg[50];

            std::sprintf(imuMsg, "i%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", mag.x, mag.y, mag.z, gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z);
            socket.send(imuMsg, sizeof imuMsg);
            
            timer.reset();
        }

        // TODO: what happens if Wi-Fi disconnects / TCP socket fails
    }
}