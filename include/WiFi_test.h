/*
 * Copyright (c) 2006-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "TCPSocket.h"
#include "ESP8266Interface.h"

#define WIFI_SSID "ssid"
#define WIFI_PASSWORD "pass"

const char *sec2str(nsapi_security_t sec);
void scan_demo(WiFiInterface *wifi);
void http_demo(NetworkInterface *net);
int test_wifi(ESP8266Interface wifi);