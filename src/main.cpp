#include "mbed.h"
#include <chrono>
#include "TCPSocket.h"
#include "ESP8266Interface.h"

// communicationvariables
ESP8266Interface wifi(PC_6, PC_7);
#define WIFI_SSID "ssid"
#define WIFI_PASSWORD "pass"
#define PORT 9000

// map variables
#define MAX_COORDS 512
int coords[MAX_COORDS] = {};

// general control variables
DigitalOut led1(LED1);
DigitalIn button(USER_BUTTON);
enum control_mode {automatic, manual} mode;
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
    if (button) {  // button is pressed
        if  (!buttonDown) {  // a new button press
            if (mode == automatic) {
                mode = manual;
            }
            else {
                mode = automatic;
            }
            buttonDown = true;     // record that the button is now down so we don't count one press lots of times
            thread_sleep_for(10);  // ignore anything for 10ms, a very basic way to de-bounce the button. 
        } 
    } else { // button isn't pressed
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

    mode = manual;

    while (true){
        switch(mode){
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

        // send battery level and coordinates every 1 second
        // TODO: put everything in a string and use one send?
        if (std::chrono::duration<float>{timer.elapsed_time()}.count() >= 1.0) {
            uint8_t batLvl = readBattery();
            socket.send(&batLvl, sizeof batLvl);
            socket.send(coords, sizeof coords);
            timer.reset();
        }

        // TODO: what happens if Wi-Fi disconnects / TCP socket fails
    }
}