#include <WiFi_interface.h>

void sendLog(TCPSocket *socket, std::string message)
{
    const uint8_t *payload = reinterpret_cast<const uint8_t *>(("l" + message).c_str());

    socket->send(payload, message.length() + 1);
}

void sendBattery(TCPSocket *socket, AnalogIn *battery_reader)
{
    uint8_t batLvl = readBattery(battery_reader);

    // send battery level
    char batMsg[3]; // one more because sprintf inserts string terminator
    std::sprintf(batMsg, "b%c", batLvl);
    socket->send(batMsg, (sizeof batMsg) - 1); // don't send string temrinator
}

void sendCoordinates(TCPSocket *socket)
{
    // send coordinates
    uint8_t coordMsg[1 + (MAX_COORDS + 2) * bytesPerCoord];
    coordMsg[0] = 'c';

    // Copy saved coordinates
    memcpy(coordMsg + 1, coords, sizeof(int) * currentCoordsSize);

    // Copy current coordinates
    memcpy(coordMsg + 1 + sizeof(int) * currentCoordsSize, position_3d, sizeof(int) * 2);

    // TODO: test again

    //socket->send(coordMsg, 1 + (currentCoordsSize + 2) * bytesPerCoord);
}

void sendIMU(TCPSocket *socket, BMI160_I2C *imu)
{
    // send IMU data
    BMI160::SensorData accData;
    BMI160::SensorData gyroData;
    BMI160::SensorData magData;
    BMI160::SensorTime sensorTime;

    imu->getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_4G, (BMI160::GyroRange)(0));
    imu->getMagSensorXYZ(magData);

    uint8_t imuMsg[1 + bytesPerIMUValue * 9];

    imuMsg[0] = 'i';
    // Each one of the three measurements occupies 3 * bytesPerIMUValue bytes
    // In the final message, the mag will start at offset 1, the gyr will start at offset
    //     1 + bytesPerIMUmeasurement, the acc will start at 1 + 2 * bytesPerIMUMeasurement
    // The offsets are based on the coordinates (x, y, z) spaced by bytesPerIMUValue
    //     and on the different measurements (mag, gyr, acc), spaced by bytesPerIMUMeasurement
    // Each of the sections will fill in one of the coordinates (x, y, z) for all three
    //     measurements.
    // "Conversions" to byte arrays are done by promoting the float to an integer by scaling it and then splitting it into bytes

    int mag_x = static_cast<int>(magData.xAxis.scaled * SCALING), mag_y = static_cast<int>(magData.yAxis.scaled * SCALING), mag_z = static_cast<int>(magData.zAxis.scaled * SCALING);
    int gyr_x = static_cast<int>(gyroData.xAxis.scaled * SCALING), gyr_y = static_cast<int>(gyroData.yAxis.scaled * SCALING), gyr_z = static_cast<int>(gyroData.zAxis.scaled * SCALING);
    int acc_x = static_cast<int>(accData.xAxis.scaled * SCALING), acc_y = static_cast<int>(accData.yAxis.scaled * SCALING), acc_z = static_cast<int>(accData.zAxis.scaled * SCALING);

    imuMsg[1] = mag_x >> 24;
    imuMsg[2] = mag_x >> 16;
    imuMsg[3] = mag_x >> 8;
    imuMsg[4] = mag_x;
    imuMsg[5] = mag_y >> 24;
    imuMsg[6] = mag_y >> 16;
    imuMsg[7] = mag_y >> 8;
    imuMsg[8] = mag_y;
    imuMsg[9] = mag_z >> 24;
    imuMsg[10] = mag_z >> 16;
    imuMsg[11] = mag_z >> 8;
    imuMsg[12] = mag_z;
    imuMsg[13] = gyr_x >> 24;
    imuMsg[14] = gyr_x >> 16;
    imuMsg[15] = gyr_x >> 8;
    imuMsg[16] = gyr_x;
    imuMsg[17] = gyr_y >> 24;
    imuMsg[18] = gyr_y >> 16;
    imuMsg[19] = gyr_y >> 8;
    imuMsg[20] = gyr_y;
    imuMsg[21] = gyr_z >> 24;
    imuMsg[22] = gyr_z >> 16;
    imuMsg[23] = gyr_z >> 8;
    imuMsg[24] = gyr_z;
    imuMsg[25] = acc_x >> 24;
    imuMsg[26] = acc_x >> 16;
    imuMsg[27] = acc_x >> 8;
    imuMsg[28] = acc_x;
    imuMsg[29] = acc_y >> 24;
    imuMsg[30] = acc_y >> 16;
    imuMsg[31] = acc_y >> 8;
    imuMsg[32] = acc_y;
    imuMsg[33] = acc_z >> 24;
    imuMsg[34] = acc_z >> 16;
    imuMsg[35] = acc_z >> 8;
    imuMsg[36] = acc_z;

    socket->send(imuMsg, sizeof imuMsg);
}

/**
 * Function to launch in a separate thread. Keeps an infinite loop reading all incoming
 * commands from the UI app and acting accordingly
 */
void readCommand(TCPSocket *socket)
{

    // Should only receive messages one byte long
    int buffer_size = 1;

    uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t) * buffer_size);

    while (socket->recv(buffer, buffer_size))
    {
        switch (buffer[0])
        {
        case FAN_ON:
        case FAN_OFF:
            if (current_mode != automatic)
            {
                fan_state = (buffer[0] & 0x40);
                printf("Fan set to %d\n", fan_state);
            }
            break;

        case AUTO_ON:
        case AUTO_OFF:
            new_mode = (buffer[0] & 0x10) ? automatic : manual;
            printf("Mode set to %s\n", (new_mode == automatic) ? "automatic" : "manual");
            break;

        case STATE_STOP:
            new_movement_state = STATE_STOP;
            printf("State set to STOP\n");
            break;

        case STATE_FORWARD:
        case STATE_BACKWARD:
        case STATE_LEFT:
        case STATE_RIGHT:
            if (new_movement_state == STATE_STOP)
            {
                new_movement_state = buffer[0];
                printf("State set to %d\n", new_movement_state);
            }
            break;

        case TEST_MODE:
            new_mode = test;
            break;
        }
    }

    /*
        Given the limited software scope of the project I think it's enough to print that the socket
        is gone and forget about it, otherwise we can return some error code and main will now that
        something is wrong and try and reconnect, but that would be a big headache since it would
        involve reconnecting to the wifi as well
    */
    printf("TCP Socket closed with an error\n");
}