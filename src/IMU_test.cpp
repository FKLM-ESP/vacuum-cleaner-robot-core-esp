#include "IMU_test.h"

int test_imu(IMU_BMX160 bmx)
{
    sBmx160SensorData_t mag, gyr, acc;

    // Get data from IMU
    bmx.getAllData(&mag, &gyr, &acc);  // You can pass 0 to the sensors you don't need
    // Display the magnetometer, gyroscope and accelerometeer results (in uT, g, m/s^2)
    printf("M X: %f Y: %f Z: %f  uT\n", mag.x, mag.y, mag.z);
    printf("G X: %f Y: %f Z: %f  g\n", gyr.x, gyr.y, gyr.z);
    printf("A X: %f Y: %f Z: %f  m/s^2\n", acc.x, acc.y, acc.z);
    printf("\n");
    return 0;
}
