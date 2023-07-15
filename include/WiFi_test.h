/*
 * Copyright (c) 2006-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Wi-Fi example from https://os.mbed.com/docs/mbed-os/v6.16/apis/wi-fi.html
 */
#include "mbed.h"
#include "TCPSocket.h"
#include "ESP8266Interface.h"

#define SSID "ExtRouter"
#define PASSWORD "easy-p@ss87"

// Send http request and expect response
void httpDemo(NetworkInterface *net);
// Test wifi by sending a http request and getting a response
int testWifi(ESP8266Interface *wifi);