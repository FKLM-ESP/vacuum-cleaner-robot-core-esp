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
    float new_ang_vel[3];

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

        imu->getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, BMI160::SENS_4G, (BMI160::GyroRange)(0));
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
        new_vel[0] = VEL_X + (
                    + 100 * accData.xAxis.raw * cos(YAW) * cos(PITCH)
                    + 100 * accData.yAxis.raw * (cos(YAW) * sin(PITCH) * sin(ROLL) - sin(YAW) * cos(ROLL))
                    + 100 * accData.zAxis.raw * (cos(YAW) * sin(PITCH) * cos(ROLL) + sin(YAW) * sin(ROLL))
                    ) * delta_s; 
        new_vel[1] = VEL_Y + (
                    + 100 * accData.xAxis.raw * sin(YAW) * cos(PITCH)
                    + 100 * accData.yAxis.raw * (sin(YAW) * sin(PITCH) * sin(ROLL) + cos(YAW) * cos(ROLL))
                    + 100 * accData.zAxis.raw * (sin(YAW) * sin(PITCH) * cos(ROLL) - cos(YAW) * sin(ROLL))
                    ) * delta_s;
        new_vel[2] = VEL_Z + (
                    - 100 * accData.xAxis.raw * sin(PITCH)
                    + 100 * accData.yAxis.raw * cos(PITCH) * sin(ROLL)
                    + 100 * accData.zAxis.raw * cos(PITCH) * cos(ROLL)
                    ) * delta_s;

        new_pos[0] = VEL_X + (
                    + new_vel[0] * cos(YAW) * cos(PITCH)
                    + new_vel[1] * (cos(YAW) * sin(PITCH) * sin(ROLL) - sin(YAW) * cos(ROLL))
                    + new_vel[2] * (cos(YAW) * sin(PITCH) * cos(ROLL) + sin(YAW) * sin(ROLL))
                    ) * delta_s; 
        new_pos[1] = VEL_Y + (
                    + new_vel[0] * sin(YAW) * cos(PITCH)
                    + new_vel[1] * (sin(YAW) * sin(PITCH) * sin(ROLL) + cos(YAW) * cos(ROLL))
                    + new_vel[2] * (sin(YAW) * sin(PITCH) * cos(ROLL) - cos(YAW) * sin(ROLL))
                    ) * delta_s;
        new_pos[2] = VEL_Z + (
                    - new_vel[0] * sin(PITCH)
                    + new_vel[1] * cos(PITCH) * sin(ROLL)
                    + new_vel[2] * cos(PITCH) * cos(ROLL)
                    ) * delta_s;

        /*
          update orientation
          NOTE: if orientation of the IMU with respect to the robot is different these values will
            will need to be swapped, or the signs changed
            (it doesn't really matter since we are not exposing these values to the user at all,
            for all we care the imu could be tilted 45° and the code would work)
            
          Sign of pitch variation is negative since it is the opposite direction of the considered
            frame of reference (see above comment and links)
        */

        new_ang_vel[0] = ANG_VEL_YAW + (gyroData.zAxis.raw * delta_s);
        new_ang_vel[1] = ANG_VEL_PITCH - (gyroData.xAxis.raw * delta_s); 
        new_ang_vel[2] = ANG_VEL_ROLL + (gyroData.yAxis.raw * delta_s);

        new_or[0] = new_ang_vel[0] + (gyroData.zAxis.raw * delta_s);
        new_or[1] = new_ang_vel[1] + (gyroData.xAxis.raw * delta_s);
        new_or[2] = new_ang_vel[2] + (gyroData.yAxis.raw * delta_s);

        memcpy(position_3d, new_pos, sizeof(int) * 3);
        memcpy(velocity_3d, new_vel, sizeof(float) * 3);
        memcpy(orientation_3d, new_or, sizeof(float) * 3);
        memcpy(ang_velocity_3d, new_ang_vel, sizeof(float) * 3);

        // Very low speed so 5ms _should_ be enough
        thread_sleep_for(5);
    }
}