#include "WiFi_test.h"

void http_demo(NetworkInterface *net)
{
    // Open a socket on the network interface, and create a TCP connection to mbed.org
    TCPSocket socket;
    socket.open(net);

    SocketAddress a;
    net->gethostbyname("ifconfig.io", &a);
    a.set_port(80);
    socket.connect(a);
    // Send a simple http request
    char sbuffer[] = "GET / HTTP/1.1\r\nHost: ifconfig.io\r\n\r\n";
    int scount = socket.send(sbuffer, sizeof sbuffer);
    printf("sent %d [%.*s]\n", scount, strstr(sbuffer, "\r\n") - sbuffer, sbuffer);

    // Recieve a simple http response and print out the response line
    char rbuffer[64];
    int rcount = socket.recv(rbuffer, sizeof rbuffer);
    printf("recv %d [%.*s]\n", rcount, strstr(rbuffer, "\r\n") - rbuffer, rbuffer);

    // Close the socket to return its memory and bring down the network interface
    socket.close();
}

int test_wifi(ESP8266Interface *wifi)
{
    SocketAddress a;

    printf("WiFi example\r\n\r\n");

    printf("\r\nConnecting to WiFi...\r\n");
    int ret = wifi->connect(SSID, PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        printf("\r\nConnection error\r\n");
        return -1;
    }

    printf("WiFi Connection Success\r\n\r\n");
    printf("MAC: %s\r\n", wifi->get_mac_address());
    wifi->get_ip_address(&a);
    printf("IP: %s\r\n", a.get_ip_address());
    wifi->get_netmask(&a);
    printf("Netmask: %s\r\n", a.get_ip_address());
    wifi->get_gateway(&a);
    printf("Gateway: %s\r\n", a.get_ip_address());
    printf("RSSI: %d\r\n\r\n", wifi->get_rssi());

    http_demo(wifi);

    wifi->disconnect();

    printf("\r\nWiFi Demo Done\r\n");
    return 0;
}