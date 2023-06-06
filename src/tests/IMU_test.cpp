#include "IMU_test.h"

DigitalOut led_imu(LED2);

int test_imu(BMI160_I2C bmx)
{   
    //check if accel and gyro are ok
    uint32_t failures = 0;
    if(bmx.setSensorPowerMode(BMI160::GYRO, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        printf("Failed to set gyroscope power mode\n");
        failures++;
    }
    thread_sleep_for(100);
    if(bmx.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        printf("Failed to set accelerometer power mode\n");
        failures++;
    }
    thread_sleep_for(100);
    if(bmx.setSensorPowerMode(BMI160::MAG, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        printf("Failed to set magnetometer power mode\n");
        failures++;
    }
    bmx.setMagnConf(); //initialize magnetometer for regular preset.
    thread_sleep_for(100);

    // //example of using getSensorConfig
    // BMI160::AccConfig accConfig;
    // if(bmx.getSensorConfig(accConfig) == BMI160::RTN_NO_ERROR) {
    //     printf("ACC Range = %d\n", accConfig.range);
    //     printf("ACC UnderSampling = %d\n", accConfig.us);
    //     printf("ACC BandWidthParam = %d\n", accConfig.bwp);
    //     printf("ACC OutputDataRate = %d\n\n", accConfig.odr);
    // } else {
    //     printf("Failed to get accelerometer configuration\n");
    //     failures++;
    // }

    // //example of setting user defined configuration
    // accConfig.range = BMI160::SENS_4G;
    // accConfig.us = BMI160::ACC_US_OFF;
    // accConfig.bwp = BMI160::ACC_BWP_2;
    // accConfig.odr = BMI160::ACC_ODR_8;
    // if(bmx.setSensorConfig(accConfig) == BMI160::RTN_NO_ERROR) {
    //     printf("ACC Range = %d\n", accConfig.range);
    //     printf("ACC UnderSampling = %d\n", accConfig.us);
    //     printf("ACC BandWidthParam = %d\n", accConfig.bwp);
    //     printf("ACC OutputDataRate = %d\n\n", accConfig.odr);
    // } else {
    //     printf("Failed to set accelerometer configuration\n");
    //     failures++;
    // }

    // BMI160::GyroConfig gyroConfig;
    // if(bmx.getSensorConfig(gyroConfig) == BMI160::RTN_NO_ERROR) {
    //     printf("GYRO Range = %d\n", gyroConfig.range);
    //     printf("GYRO BandWidthParam = %d\n", gyroConfig.bwp);
    //     printf("GYRO OutputDataRate = %d\n\n", gyroConfig.odr);
    // } else {
    //     printf("Failed to get gyroscope configuration\n");
    //     failures++;
    // }


    thread_sleep_for(1000);
    if(failures == 0) {
        float imuTemperature;
        BMI160::SensorData accData;
        BMI160::SensorData gyroData;
        BMI160::SensorData magData;
        BMI160::SensorTime sensorTime;

        for(int i = 0; i < 5; i++) {
            bmx.getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_4G, (BMI160::GyroRange)(0));

            printf("ACC xAxis (g) = %4.3f\n", accData.xAxis.scaled);
            printf("ACC yAxis (g) = %4.3f\n", accData.yAxis.scaled);
            printf("ACC zAxis (g) = %4.3f\n\n", accData.zAxis.scaled);

            printf("GYRO xAxis (dps) = %5.1f\n", gyroData.xAxis.scaled);
            printf("GYRO yAxis (dps) = %5.1f\n", gyroData.yAxis.scaled);
            printf("GYRO zAxis (dps) = %5.1f\n\n",gyroData.zAxis.scaled);

            if (accData.zAxis.scaled > -5.0 || accData.zAxis.scaled < 5.0)
            {
                led_imu = 1;
            }

            bmx.getMagSensorXYZ(magData);

            printf("Mag xAxis (uT) = %5.1f\n", magData.xAxis.scaled);
            printf("Mag yAxis (uT) = %5.1f\n", magData.yAxis.scaled);
            printf("Mag zAxis (uT) = %5.1f\n\n",magData.zAxis.scaled);

            bmx.getTemperature(&imuTemperature);

            printf("Sensor Time = %f\n",sensorTime.seconds);
            printf("Sensor Temperature = %5.3f\n",  imuTemperature);

            thread_sleep_for(500);


        }
    } else {
        while(1) {
            printf("Error\n");
        }
    }

    return 0;
}
