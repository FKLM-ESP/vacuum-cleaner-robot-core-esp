#include "WiFi_interface.h"
#include "mbed.h"
#include "TCPSocket.h"
#include "variables.h"
#include "ESP8266Interface.h"

int connect_to_wifi(ESP8266Interface *wifi);
bool connect_to_socket(ESP8266Interface *wifi, TCPSocket *socket);