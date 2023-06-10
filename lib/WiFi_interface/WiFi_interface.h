#include "TCPSocket.h"
#include "Battery.h"
#include "variables.h"


int currentCoordsSize
int *coords

void sendLog(TCPSocket *socket, std::string log_message);
void sendBattery(TCPSocket *socket);
void sendCoordinates(TCPSocket *socket);
void sendIMU(TCPSocket *socket, BMI160_I2C* imu);