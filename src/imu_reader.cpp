#include <imu_reader.h>

/**
 * This function will be launched in a thread at the beginnin of main()
 * It will enter an infinite loop, in which it will read values from the imu and
 *     update current position and orientation.
 */

void imu_read_and_update_coords(BMI160_I2C *imu)
{
    // send IMU data
    BMI160::SensorData accData;
    BMI160::SensorData gyroData;
    BMI160::SensorData magData;
    BMI160::SensorTime sensorTime;

    // Calculate IMU pose with respect to the ground
    imu->getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_4G, (BMI160::GyroRange)(0));
    imu->getMagSensorXYZ(magData);

    float Rx = accData.xAxis.scaled / 2 * GRAVITY;
    float Ry = accData.yAxis.scaled / 2 * GRAVITY;
    float Rz = accData.zAxis.scaled / 2 * GRAVITY;

    // should be around 9.81, magnitude of the local g value
    float R = sqrt(Rx * Rx +
                   Ry * Ry +
                   Rz * Rz);

    // estimated new values
    float Rx_est, Ry_est, Rz_est;

    // estimated values at previous step
    float Rx_est_old = Rx;
    float Ry_est_old = Ry;
    float Rz_est_old = Rz;
    
    // util variables
    float Rate_Axz_avg, Rate_Ayz_avg, Rate_Axy_avg;

    // estimated angles at previous step
    float Axz_old, Ayz_old, Axy_old;

    // estimated angles at current step
    float Axz, Ayz, Axy;

    // pose of the IMU with respect to earth's gravity
    float Axz_0 = atan2f(Rx_est_old, Rz_est_old);
    float Ayz_0 = atan2f(Ry_est_old, Rz_est_old);
    float Axy_0 = atan2f(Rx_est_old, Ry_est_old);

    // accelerations calculated from the gyro
    float Rx_gyro;
    float Ry_gyro;
    float Rz_gyro;

    // values of acceleration in the global frame of reference
    float Accx_glob, Accy_glob, Accz_glob;

    // calculated output vectors
    int * new_pos;
    float * new_vel;
    float * new_or;

    // old vectors to free them later
    int * old_pos;
    float * old_vel;
    float * old_or;

    Timer timer;
    timer.start();

    std::chrono::_V2::system_clock::time_point last_time = std::chrono::high_resolution_clock::now();

    std::chrono::_V2::system_clock::time_point new_time;
    float delta_s;

    while (true)
    {

        new_pos = (int *)malloc(sizeof(int) * 3);
        new_vel = (float *)malloc(sizeof(float) * 3);
        new_or = (float *)malloc(sizeof(float) * 3);

        new_time = std::chrono::high_resolution_clock::now();
        delta_s = (std::chrono::duration<float>(new_time - last_time)).count();

        last_time = new_time;

        imu->getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_2G, (BMI160::GyroRange)(0));
        imu->getMagSensorXYZ(magData);

        Rx = accData.xAxis.scaled / 2 * GRAVITY;
        Ry = accData.yAxis.scaled / 2 * GRAVITY;
        Rz = accData.zAxis.scaled / 2 * GRAVITY;

        // Calculate new orientation of the imu
        Axz_old = atan2f(Rx_est_old, Rz_est_old);
        Rate_Axz_avg = (Axz_old + gyroData.yAxis.scaled / 180 * PI) / 2;
        Axz = Axz_old + Rate_Axz_avg * delta_s;

        Ayz_old = atan2f(Ry_est_old, Rz_est_old);
        Rate_Ayz_avg = (Ayz_old + gyroData.xAxis.scaled / 180 * PI) / 2;
        Ayz = Ayz_old - Rate_Ayz_avg * delta_s;

        Axy_old = atan2f(Rx_est_old, Ry_est_old);
        Rate_Axy_avg = (Axy_old + gyroData.zAxis.scaled / 180 * PI) / 2;
        Axy = Axy_old - Rate_Axy_avg * delta_s;

        // Use orientation values to calculate orientation using gyro values
        Rx_gyro = sin(Axz) / sqrt(1 + pow(cos(Axz), 2) * pow(tan(Ayz), 2));
        Ry_gyro = sin(Ayz) / sqrt(1 + pow(cos(Ayz), 2) * pow(tan(Axz), 2));
        Rz_gyro = ((Rz_est_old >= 0) ? 1 : -1) * sqrt(1 - pow(Rx_gyro, 2) - pow(Ry_gyro, 2));

         new_or[0] = YAW + (gyroData.zAxis.scaled / 180 * PI * delta_s);
        // Keep it in range. We don't care for pitch and roll (if imu is flat enough on the robot)
        if (new_or[0] > 6.28319)
        {
          new_or[0] -= 6.28319;
        }
        else if (new_or[0] < 0)
        {
          new_or[0] += 6.28319;
        }
        new_or[1] = PITCH - (gyroData.xAxis.scaled / 180 * PI * delta_s);
        new_or[2] = ROLL + (gyroData.yAxis.scaled / 180 * PI * delta_s);

        if (Rx_gyro == Rx_gyro && Ry_gyro == Ry_gyro && Rz_gyro == Rz_gyro)
        {
            // guide says something between 5 and 20, means how much you trust the gyro
            float w_gyro = 15;

            // estimate accelerations in the IMU frame of reference
            Rx_est = (Rx + Rx_gyro * w_gyro) / (1 + w_gyro);
            Ry_est = (Ry + Ry_gyro * w_gyro) / (1 + w_gyro);
            Rz_est = (Rz + Rz_gyro * w_gyro) / (1 + w_gyro);

            Rx_est_old = Rx_est;
            Ry_est_old = Ry_est;
            Rz_est_old = Rz_est;

            // // Project accelerations into global frame of reference (is it necessary???)
            // Accx_glob = Rx_est *  cos(Axy) * cos(Ayz)
            //             + Ry_est * (cos(Axy) * sin(Ayz) * sin(Axz) - sin(Axy) * cos(Axz))
            //             + Rz_est * (cos(Axy) * sin(Ayz) * cos(Axz) + sin(Axy) * sin(Axz));
            // Accy_glob =  Rx_est * sin(Axy) * cos(Ayz)
            //             + Ry_est * (sin(Axy) * sin(Ayz) * sin(Axz) + cos(Axy) * cos(Axz))
            //             + Rz_est * (sin(Axy) * sin(Ayz) * cos(Axz) - cos(Axy) * sin(Axz));
            // Accz_glob = Rx_est * sin(Ayz)
            //             + Ry_est * cos(Ayz) * sin(Axz)
            //             + Rz_est * cos(Ayz) * cos(Axz);

            // Project accelerations into global frame of reference (is it necessary???)
            Accx_glob = Rx_est *  cos(YAW) * cos(PITCH)
                        + Ry_est * (cos(YAW) * sin(PITCH) * sin(ROLL) - sin(YAW) * cos(ROLL))
                        + Rz_est * (cos(YAW) * sin(PITCH) * cos(ROLL) + sin(YAW) * sin(ROLL));
            Accy_glob =  Rx_est * sin(YAW) * cos(PITCH)
                        + Ry_est * (sin(YAW) * sin(PITCH) * sin(ROLL) + cos(YAW) * cos(ROLL))
                        + Rz_est * (sin(YAW) * sin(PITCH) * cos(ROLL) - cos(YAW) * sin(ROLL));
            Accz_glob = Rx_est * sin(PITCH)
                        + Ry_est * cos(PITCH) * sin(ROLL)
                        + Rz_est * cos(PITCH) * cos(ROLL);
            // otherwise
            // Accx_glob = Rx_est;
            // Accy_glob = Ry_est;
            // Accz_glob = Rz_est;


            // double integrate to get new velocity and acceleration
            new_vel[0] = VEL_X + 100 * Accx_glob * delta_s;
            new_vel[1] = VEL_Y + 100 * Accy_glob * delta_s;
            new_vel[2] = VEL_Z + 100 * (Accz_glob - 1.54) * delta_s;

            new_pos[0] = POS_X + new_vel[0] * delta_s;
            new_pos[1] = POS_Y + new_vel[1] * delta_s;
            new_pos[2] = POS_Z + new_vel[2] * delta_s;

            old_vel = velocity_3d;
            velocity_3d = new_vel;
            free(old_vel);

            old_pos = position_3d;
            position_3d = new_pos;
            free(old_pos);
        }

        old_or = orientation_3d;
        orientation_3d = new_or;
        free(old_or);
       
        // Very low speed so 5ms _should_ be enough
        thread_sleep_for(5);
    }
}