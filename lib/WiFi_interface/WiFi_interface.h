#include "mbed.h"
#include "TCPSocket.h"
#include <Battery.h>
#include <variables.h>

#include <bmx160.h>

void sendLog(TCPSocket *socket, std::string log_message);
void sendBattery(TCPSocket *socket, AnalogIn *battery_reader);
void sendCoordinates(TCPSocket *socket);
void sendIMU(TCPSocket *socket, BMI160_I2C* imu);