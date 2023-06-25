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

    int new_pos[3];
    float new_vel[3];
    float new_or[3];

    Timer timer;
    timer.start();

    std::chrono::_V2::system_clock::time_point last_time = std::chrono::high_resolution_clock::now();

    std::chrono::_V2::system_clock::time_point new_time;
    float delta_s;

    while (true)
    {
        new_time = std::chrono::high_resolution_clock::now();
        delta_s = (std::chrono::duration<float>(new_time - last_time)).count();

        last_time = new_time;

        imu->getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_2G, (BMI160::GyroRange)(0));
        imu->getMagSensorXYZ(magData);

        /*
          TODO: confirm orientation with the sensor mounted on the robot

          For reference, the dot on chip is between the x and y axes (on the right and the left of the dot resp.)
          It doesn't really matter since it only changes the initial orientation of the map, which is already
            random depending on how the robot is placed on the ground.
          The following code (actually the convention for YAW, PITCH and ROLL), assumes thath x points to the right,
            y to the front and z to the top
          
          references for the orientation of the axes and the matrix: (yes, from Steven lol)
            https://msl.cs.uiuc.edu/planning/node102.html
            https://msl.cs.uiuc.edu/planning/img814.gif
        */
        // new_vel[0] = VEL_X + (
        //             + accData.xAxis.scaled * GRAVITY_MULTIPLIER *  cos(YAW) * cos(PITCH)
        //             + accData.yAxis.scaled * GRAVITY_MULTIPLIER * (cos(YAW) * sin(PITCH) * sin(ROLL) - sin(YAW) * cos(ROLL))
        //             + (accData.zAxis.scaled - GRAVITY) * GRAVITY_MULTIPLIER * (cos(YAW) * sin(PITCH) * cos(ROLL) + sin(YAW) * sin(ROLL))
        //             ) * delta_s; 
        // new_vel[1] = VEL_Y + (
        //             + accData.xAxis.scaled * GRAVITY_MULTIPLIER * sin(YAW) * cos(PITCH)
        //             + accData.yAxis.scaled * GRAVITY_MULTIPLIER * (sin(YAW) * sin(PITCH) * sin(ROLL) + cos(YAW) * cos(ROLL))
        //             + (accData.zAxis.scaled - GRAVITY) * GRAVITY_MULTIPLIER * (sin(YAW) * sin(PITCH) * cos(ROLL) - cos(YAW) * sin(ROLL))
        //             ) * delta_s;
        // new_vel[2] = VEL_Z + (
        //             - accData.xAxis.scaled * GRAVITY_MULTIPLIER * sin(PITCH)
        //             + accData.yAxis.scaled * GRAVITY_MULTIPLIER * cos(PITCH) * sin(ROLL)
        //             + (accData.zAxis.scaled - GRAVITY) * GRAVITY_MULTIPLIER * cos(PITCH) * cos(ROLL)
        //             ) * delta_s;

        new_vel[0] = VEL_X + accData.xAxis.scaled * delta_s * GRAVITY_MULTIPLIER;
        new_vel[1] = VEL_Y + accData.yAxis.scaled * delta_s * GRAVITY_MULTIPLIER;
        new_vel[2] = VEL_Z + (accData.zAxis.scaled * GRAVITY_MULTIPLIER - GRAVITY) * delta_s;

        new_pos[0] = POS_X + new_vel[0] * delta_s;
        new_pos[1] = POS_Y + new_vel[1] * delta_s;
        new_pos[2] = POS_Z + new_vel[2] * delta_s;
        // new_pos[0] = VEL_X + (
        //             + new_vel[0] * cos(YAW) * cos(PITCH)
        //             + new_vel[1] * (cos(YAW) * sin(PITCH) * sin(ROLL) - sin(YAW) * cos(ROLL))
        //             + new_vel[2] * (cos(YAW) * sin(PITCH) * cos(ROLL) + sin(YAW) * sin(ROLL))
        //             ) * delta_s; 
        // new_pos[1] = VEL_Y + (
        //             + new_vel[0] * sin(YAW) * cos(PITCH)
        //             + new_vel[1] * (sin(YAW) * sin(PITCH) * sin(ROLL) + cos(YAW) * cos(ROLL))
        //             + new_vel[2] * (sin(YAW) * sin(PITCH) * cos(ROLL) - cos(YAW) * sin(ROLL))
        //             ) * delta_s;
        // new_pos[2] = VEL_Z + (
        //             - new_vel[0] * sin(PITCH)
        //             + new_vel[1] * cos(PITCH) * sin(ROLL)
        //             + new_vel[2] * cos(PITCH) * cos(ROLL)
        //             ) * delta_s;

        /*
          update orientation
          NOTE: if orientation of the IMU with respect to the robot is different these values will
            will need to be swapped, or the signs changed
            (it doesn't really matter since we are not exposing these values to the user at all,
            for all we care the imu could be tilted 45° and the code would work)
            
          Sign of pitch variation is negative since it is the opposite direction of the considered
            frame of reference (see above comment and links)
        */

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

        //printf("Acc_x: %2.4f\tAcc_y: %2.4f\tAcc_z: %2.4f\n", accData.xAxis.scaled * GRAVITY_MULTIPLIER, accData.yAxis.scaled * GRAVITY_MULTIPLIER, accData.zAxis.scaled * GRAVITY_MULTIPLIER);

        memcpy(position_3d, new_pos, sizeof(int) * 3);
        memcpy(velocity_3d, new_vel, sizeof(float) * 3);
        memcpy(orientation_3d, new_or, sizeof(float) * 3);

        // Very low speed so 5ms _should_ be enough
        thread_sleep_for(5);
    }
}