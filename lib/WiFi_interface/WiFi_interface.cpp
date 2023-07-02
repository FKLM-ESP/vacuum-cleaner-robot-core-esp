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
    for(int i = 0; i < currentCoordsSize; i++)
    {
        coordMsg[1 + i * sizeof(int) + 0] = coords[i] >> 24;
        coordMsg[1 + i * sizeof(int) + 1] = coords[i] >> 16;
        coordMsg[1 + i * sizeof(int) + 2] = coords[i] >> 8;
        coordMsg[1 + i * sizeof(int) + 3] = coords[i];
    }

    // Copy current coordinates
    coordMsg[1 + currentCoordsSize * sizeof(int) + 0] = POS_X >> 24;
    coordMsg[1 + currentCoordsSize * sizeof(int) + 1] = POS_X >> 16;
    coordMsg[1 + currentCoordsSize * sizeof(int) + 2] = POS_X >> 8;
    coordMsg[1 + currentCoordsSize * sizeof(int) + 3] = POS_X;

    coordMsg[1 + currentCoordsSize * sizeof(int) + 4] = POS_Y >> 24;
    coordMsg[1 + currentCoordsSize * sizeof(int) + 5] = POS_Y >> 16;
    coordMsg[1 + currentCoordsSize * sizeof(int) + 6] = POS_Y >> 8;
    coordMsg[1 + currentCoordsSize * sizeof(int) + 7] = POS_Y;

    socket->send(coordMsg, 1 + (currentCoordsSize + 2) * bytesPerCoord);
}

void sendIMU(TCPSocket *socket, BMI160_I2C *imu)
{
    // send IMU data
    BMI160::SensorData accData;
    BMI160::SensorData gyroData;
    BMI160::SensorData magData;
    BMI160::SensorTime sensorTime;

    imu->getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_2G, (BMI160::GyroRange)(0));
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
    int gyr_x = static_cast<int>(gyroData.xAxis.scaled / 180 * PI * SCALING), gyr_y = static_cast<int>(gyroData.yAxis.scaled / 180 * PI * SCALING), gyr_z = static_cast<int>(gyroData.zAxis.scaled / 180 * PI * SCALING);
    int acc_x = static_cast<int>(accData.xAxis.scaled * GRAVITY_MULTIPLIER * SCALING), acc_y = static_cast<int>(accData.yAxis.scaled * GRAVITY_MULTIPLIER * SCALING), acc_z = static_cast<int>(accData.zAxis.scaled * GRAVITY_MULTIPLIER * SCALING);

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

    
    // printf("GYRO xAxis (dps) = %5.1f\n", gyroData.xAxis.scaled / 180 * PI);
    // printf("GYRO yAxis (dps) = %5.1f\n", gyroData.yAxis.scaled / 180 * PI);
    // printf("GYRO zAxis (dps) = %5.1f\n\n", gyroData.zAxis.scaled / 180 * PI);

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

    uint8_t buffer = 0;

    nsapi_size_or_error_t ret = socket->recv(&buffer, buffer_size);
    if (ret > 0)
    {
        switch (buffer)
        {
        case FAN_ON:
        case FAN_OFF:
            if (current_mode == manual)
            {
                fan_state = (buffer & 0x40);
                printf("Fan set to %d\n", fan_state);
            }
            break;

        case AUTO_ON:
        case AUTO_OFF:
            new_mode = (buffer & 0x10) ? automatic : manual;
            printf("Mode set to %s\n", (new_mode == automatic) ? "automatic" : "manual");
            break;

        case STATE_STOP:
            if (current_mode == manual)
            {
                new_movement_state = STATE_STOP;
                printf("State set to STOP\n");
            }
            break;

        case STATE_FORWARD:
        case STATE_BACKWARD:
        case STATE_LEFT:
        case STATE_RIGHT:
            if (current_movement_state == STATE_STOP && current_mode == manual)
            {
                new_movement_state = buffer;
                printf("State set to %d\n", new_movement_state);
            }
            break;

        case TEST_MODE:
            new_mode = test;
            break;
        }
    }
    else
    {
        if (ret != NSAPI_ERROR_WOULD_BLOCK && ret != NSAPI_ERROR_OK)
            printf("TCP Socket closed with error, %d\n", ret);
    }

}