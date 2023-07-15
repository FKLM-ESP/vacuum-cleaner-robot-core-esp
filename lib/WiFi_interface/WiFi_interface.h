#include "mbed.h"
#include "TCPSocket.h"
#include <Battery.h>
#include <variables.h>

#include <bmx160.h>

// Send logs to app
void sendLog(TCPSocket *socket, std::string log_message);

// Send battery level to app
void sendBattery(TCPSocket *socket, AnalogIn *battery_reader);

// Send coordinates to app
void sendCoordinates(TCPSocket *socket);

// Send IMU data to app
void sendIMU(TCPSocket *socket, BMI160_I2C* imu);

// Read all incoming commands from the UI app and act accordingly
void readCommand(TCPSocket *socket);