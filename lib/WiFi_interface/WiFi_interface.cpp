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

    socket->send(coordMsg, 1 + (currentCoordsSize + 2) * bytesPerCoord);
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

    //uint8_t imuMsg[1 + bytesPerIMUValue * 9];
    uint8_t imuMsg[1 + bytesPerIMUValue * 9];

    imuMsg[0] = 'i';
    // Each one of the three measurements occupies 3 * bytesPerIMUValue bytes
    // In the final message, the mag will start at offset 1, the gyr will start at offset
    //     1 + bytesPerIMUmeasurement, the acc will start at 1 + 2 * bytesPerIMUMeasurement
    // The offsets are based on the coordinates (x, y, z) spaced by bytesPerIMUValue
    //     and on the different measurements (mag, gyr, acc), spaced by bytesPerIMUMeasurement
    // Each of the sections will fill in one of the coordinates (x, y, z) for all three
    //     measurements.
    // "Conversions" to byte arrays are done only once before the loop

    int bytesPerIMUMeasurement = 3 * bytesPerIMUValue;

    // int magOffset = 0;
    // int gyrOffset = bytesPerIMUMeasurement;
    // int accOffset = 2 * bytesPerIMUMeasurement;
    // int xOffset = 0;
    // int yOffset = bytesPerIMUValue;
    // int zOffset = 2 * bytesPerIMUValue;

    // printf("ACC xAxis (g) = %4.3f\n", accData.xAxis.scaled);
    // printf("ACC yAxis (g) = %4.3f\n", accData.yAxis.scaled);
    // printf("ACC zAxis (g) = %4.3f\n\n", accData.zAxis.scaled);

    // uint8_t *mag_x = (uint8_t *)(&(magData.xAxis.scaled)), *mag_y = (uint8_t *)(&(magData.yAxis.scaled)), *mag_z = (uint8_t *)(&(magData.zAxis.scaled));
    // uint8_t *gyr_x = (uint8_t *)(&(gyroData.xAxis.scaled)), *gyr_y = (uint8_t *)(&(gyroData.yAxis.scaled)), *gyr_z = (uint8_t *)(&(gyroData.zAxis.scaled));
    // uint8_t *acc_x = (uint8_t *)(&(accData.xAxis.scaled)), *acc_y = (uint8_t *)(&(accData.yAxis.scaled)), *acc_z = (uint8_t *)(&(accData.zAxis.scaled));

    // the loop iterates over the indices for a single value (e.g. 0,1,2,3 for a float) and fills all the
    //     corresponding byte for all coordinates of all measurements
    // for (int i = 0; i < bytesPerIMUValue; i++)
    // {
        // imuMsg[1 + i + xOffset + magOffset] = mag_x[i];
        // imuMsg[1 + i + xOffset + gyrOffset] = gyr_x[i];
        // imuMsg[1 + i + xOffset + accOffset] = acc_x[i];


        // imuMsg[1 + i + yOffset + magOffset] = mag_y[i];
        // imuMsg[1 + i + yOffset + gyrOffset] = gyr_y[i];
        // imuMsg[1 + i + yOffset + accOffset] = acc_y[i];

        // imuMsg[1 + i + zOffset + magOffset] = mag_z[i];
        // imuMsg[1 + i + zOffset + gyrOffset] = gyr_z[i];
        // imuMsg[1 + i + zOffset + accOffset] = acc_z[i];
        
        // printf("ACC xAxis (g) = %d\n", acc_x[i]);
        // printf("ACC yAxis (g) = %d\n", acc_y[i]);
        // printf("ACC zAxis (g) = %d\n\n", acc_z[i]);
    //}

    // for (int i = 0; i < 9 * bytesPerIMUValue; i ++)
    // {
    //     printf("%hhx ", imuMsg[i]);
    // }
    // printf("\n");

    int mag_x = magData.xAxis.scaled * 10000, mag_y = magData.yAxis.scaled * 10000, mag_z = magData.zAxis.scaled * 10000;
    int gyr_x = gyroData.xAxis.scaled * 10000, gyr_y = gyroData.yAxis.scaled * 10000, gyr_z = gyroData.zAxis.scaled * 10000;
    int acc_x = accData.xAxis.scaled * 10000, acc_y = accData.yAxis.scaled * 10000, acc_z = accData.zAxis.scaled * 10000;

    memcpy(imuMsg + 1, &mag_x, 4);
    memcpy(imuMsg + 5, &mag_y, 4);
    memcpy(imuMsg + 9, &mag_z, 4);
    memcpy(imuMsg + 13, &gyr_x, 4);
    memcpy(imuMsg + 17, &gyr_y, 4);
    memcpy(imuMsg + 21, &gyr_z, 4);
    memcpy(imuMsg + 25, &acc_x, 4);
    memcpy(imuMsg + 29, &acc_y, 4);
    memcpy(imuMsg + 33, &acc_z, 4);

    for (int i = 1; i < 9  * bytesPerIMUValue; i += bytesPerIMUValue)
    {
        printf("%d ", imuMsg[i]);
    }
    printf("\n");

    
    // TODO: test
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