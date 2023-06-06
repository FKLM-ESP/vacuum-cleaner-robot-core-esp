#include "mbed.h"
#include "TCPSocket.h"

void sendIMU(TCPSocket *socket);
void sendBattery(TCPSocket *socket);
void sendCoordinates(TCPSocket *socket);