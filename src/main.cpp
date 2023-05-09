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
const int bytesPerCoord = sizeof(int);
int currentCoordsSize = 0; // increase by two with each new coordinate
int coords[MAX_COORDS] = {};

const int bytesPerIMUValue = sizeof(float);

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

uint8_t readBattery()
{
    // TODO: read battery level
    return 100;
}

void sendBattery(TCPSocket *socket)
{
    uint8_t batLvl = readBattery();

    // send battery level
    char batMsg[3]; // one more because sprintf inserts string terminator
    std::sprintf(batMsg, "b%c", batLvl);
    socket->send(batMsg, (sizeof batMsg) - 1); // don't send string temrinator
}

void sendCoordinates(TCPSocket *socket)
{
    // send coordinates
    uint8_t coordMsg[1 + MAX_COORDS * bytesPerCoord];
    coordMsg[0] = 'c';
    for (int i = 0; i <= currentCoordsSize * bytesPerCoord; i++)
    {
        // Dump bytes of coords into coodsMsg
        coordMsg[i + 1] = coords[i];
    }

    // TODO: test below line
    socket->send(coordMsg, 1 + currentCoordsSize * bytesPerCoord);
}

void sendIMU(TCPSocket *socket)
{
    // send IMU data
    sBmx160SensorData_t mag, gyr, acc;
    imu.getAllData(&mag, &gyr, &acc);
    uint8_t imuMsg[1 + bytesPerIMUValue * 9];

    imuMsg[0] = 'i';
    // Each one of the three measurements occupies 3 * bytesPerIMUValue bytes
    // In the final message, the mag will start at offset 1, the gyr will start at offset
    //     1 + bytesPerIMUmeasurement, the acc will start at 1 + 2 * bytesPerIMUMeasurement
    // The offsets are based on the coordinates (x, y, z) spaced by bytesPerIMUValue
    //     and on the different measurements (mag, gyr, acc), spaced by bytesPerIMUMeasurement
    // Each of the sections will fill in one of the coordinates (x, y, z) for all three
    //     measurements.
    // "Conversions" to byte arrays are done only once before the loop

    int bytesPerIMUMeasurement = 3 * bytesPerIMUValue;

    int magOffset = 0;
    int gyrOffset = bytesPerIMUMeasurement;
    int accOffset = 2 * bytesPerIMUMeasurement;
    int xOffset = 0;
    int yOffset = bytesPerIMUValue;
    int zOffset = 2 * bytesPerIMUValue;

    uint8_t *mag_x = (uint8_t *)(&(mag.x)), *mag_y = (uint8_t *)(&(mag.y)), *mag_z = (uint8_t *)(&(mag.z));
    uint8_t *gyr_x = (uint8_t *)(&(gyr.x)), *gyr_y = (uint8_t *)(&(gyr.y)), *gyr_z = (uint8_t *)(&(gyr.z));
    uint8_t *acc_x = (uint8_t *)(&(acc.x)), *acc_y = (uint8_t *)(&(acc.y)), *acc_z = (uint8_t *)(&(acc.z));

    // the loop iterates over the indices for a single value (e.g. 0,1,2,3 for a float) and fills all the
    //     corresponding byte for all coordinates of all measurements
    for (int i = 0; i < bytesPerIMUValue; i++)
    {
        imuMsg[1 + i + xOffset + magOffset] = mag_x[i];
        imuMsg[1 + i + xOffset + gyrOffset] = gyr_x[i];
        imuMsg[1 + i + xOffset + accOffset] = acc_x[i];

        imuMsg[1 + i + yOffset + magOffset] = mag_y[i];
        imuMsg[1 + i + yOffset + gyrOffset] = gyr_y[i];
        imuMsg[1 + i + yOffset + accOffset] = acc_y[i];

        imuMsg[1 + i + zOffset + magOffset] = mag_z[i];
        imuMsg[1 + i + zOffset + gyrOffset] = gyr_z[i];
        imuMsg[1 + i + zOffset + accOffset] = acc_z[i];
    }

    socket->send(imuMsg, sizeof imuMsg);
}

void autoClean()
{
    // TODO: handle automatic cleaning algorithm
}

void handleControls()
{
    // TODO: handle control commands sent from GUI
}

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
        switch (mode)
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
            sendBattery(&socket);

            sendCoordinates(&socket);

            sendIMU(&socket);

            timer.reset();
        }

        // TODO: what happens if Wi-Fi disconnects / TCP socket fails
    }
}