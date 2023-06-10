#include "TCPSocket.h"
#include "Battery.h"
#include "variables.h"


int currentCoordsSize
int *coords

void sendLog(TCPSocket *socket, std::string log_message);
void sendIMU(TCPSocket *socket);