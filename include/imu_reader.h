#include <variables.h>
#include <types.h>
#include <bmx160.h>

extern BMI160::SensorData accData;
extern BMI160::SensorData gyroData;
extern BMI160::SensorData magData;
extern BMI160::SensorTime sensorTime;

// values after conversion
extern float Rx, Ry, Rz;

// estimated new values
extern float Rx_est, Ry_est, Rz_est;

// estimated values at previous step
extern float Rx_est_old;
extern float Ry_est_old;
extern float Rz_est_old;

// util variables
extern float Rate_Axz_avg, Rate_Ayz_avg, Rate_Axy_avg;

// estimated angles at previous step
extern float Axz_old, Ayz_old, Axy_old;

// estimated angles at current step
extern float Axz, Ayz, Axy;

// accelerations calculated from the gyro
extern float Rx_gyro;
extern float Ry_gyro;
extern float Rz_gyro;

// values of acceleration in the global frame of reference
extern float Accx_glob, Accy_glob, Accz_glob;

// calculated output vectors
extern int new_pos[3];
extern float new_vel[3];
extern float new_or[3];

extern std::chrono::_V2::system_clock::time_point last_time;
extern std::chrono::_V2::system_clock::time_point new_time;
extern float delta_s;

void imu_read_and_update_coords(BMI160_I2C * imu);