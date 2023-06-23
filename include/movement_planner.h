#include "mbed.h"
#include "variables.h"
#include "Ultrasonic.h"
#include <stdlib.h>
#include "WiFi_interface.h"

//void autoClean(Ultrasonic *sensor_1, Ultrasonic *sensor_2, TCPSocket *socket);
void autoClean(Ultrasonic *sensor_1, Ultrasonic *sensor_2, TCPSocket *wifi);