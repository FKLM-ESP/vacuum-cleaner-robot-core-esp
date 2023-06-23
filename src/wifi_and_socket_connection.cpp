#include <wifi_and_socket_connection.h>

int connect_to_wifi()
{
    // Connect to Wi-Fi
    printf("\r\nConnecting...\r\n");
    int ret = wifi.connect(WIFI_SSID, WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0)
    {
        printf("\r\nCan't connect to wi-fi\r\n");
    }
    else
    {
        printf("Connected to WiFi!\r\n\r\n");
    }
    return ret;
}

bool connect_to_socket()
{
    SocketAddress a;
    socket.open(&wifi);
    wifi.gethostbyname(IP_ADDR, &a); // address might be device dependent ???
    a.set_port(PORT);
    nsapi_error_t ret = socket.connect(a);

    if (ret == NSAPI_ERROR_OK && ret != NSAPI_ERROR_IS_CONNECTED)
    {
        printf("Connected to socket!\r\n\r\n");
    }
    else if (ret != NSAPI_ERROR_IS_CONNECTED)
    {
        printf("\r\nCan't connect to UI app socket\r\n");
    }

    return ret == NSAPI_ERROR_OK;
}