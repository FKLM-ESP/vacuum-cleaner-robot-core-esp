/*
  This file is a direct port of the DFRobot BMX160 library available for
  Arduino systems at https://github.com/DFRobot/DFRobot_BMX160.

  Changes from the original library:
   - Changed header file import name
   - changed class name from DFRobot_BMX160 to IMU_BMX160
   - changed all delay calls to thread_sleep_for, with same arguments
   - rewritten writeReg, readReg and scan class methods using methods of I2C class
*/

/*
USAGE:

#include "IMU_BMX160.h"

// imu object ready to use
I2C imu_i2c(I2C_SDA, I2C_SCL);
IMU_BMX160 imu(&imu_i2c);

sBmx160SensorData_t mag, gyr, acc;

// Get data from IMU
imu.getAllData(&mag, &gyr, &acc);  // You can pass 0 to the sensors you don't need
// Display the magnetometer, gyroscope and accelerometeer results (in uT, g, m/s^2)
printf("M X: %f  %f  %f  uT\n", mag.x, mag.y, mag.z);
printf("G X: %f  %f  %f  g\n", gyr.x, gyr.y, gyr.z);
printf("A X: %f  %f  %f  m/s^2\n", acc.x, acc.y, acc.z);
printf("\n");

*/

#include "IMU_BMX160.h"

IMU_BMX160::IMU_BMX160(I2C *pWire)
{
    _pWire = pWire;
    Obmx160 = (sBmx160Dev_t *)malloc(sizeof(sBmx160Dev_t));
    Oaccel = (sBmx160SensorData_t *)malloc(sizeof(sBmx160SensorData_t));
    Ogyro = (sBmx160SensorData_t *)malloc(sizeof(sBmx160SensorData_t));
    Omagn = (sBmx160SensorData_t *)malloc(sizeof(sBmx160SensorData_t));
}

const uint8_t int_mask_lookup_table[13] = {
    BMX160_INT1_SLOPE_MASK,
    BMX160_INT1_SLOPE_MASK,
    BMX160_INT2_LOW_STEP_DETECT_MASK,
    BMX160_INT1_DOUBLE_TAP_MASK,
    BMX160_INT1_SINGLE_TAP_MASK,
    BMX160_INT1_ORIENT_MASK,
    BMX160_INT1_FLAT_MASK,
    BMX160_INT1_HIGH_G_MASK,
    BMX160_INT1_LOW_G_MASK,
    BMX160_INT1_NO_MOTION_MASK,
    BMX160_INT2_DATA_READY_MASK,
    BMX160_INT2_FIFO_FULL_MASK,
    BMX160_INT2_FIFO_WM_MASK};

bool IMU_BMX160::start()
{
    _pWire->start();
    if (scan() == true)
    {
        softReset();
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
        thread_sleep_for(50);
        /* Set gyro to normal mode */
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
        thread_sleep_for(100);
        /* Set mag to normal mode */
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
        thread_sleep_for(10);
        setMagnConf();
        return true;
    }
    else
        return false;
}

void IMU_BMX160::setLowPower()
{
    softReset();
    thread_sleep_for(100);
    setMagnConf();
    thread_sleep_for(100);
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x12);
    thread_sleep_for(100);
    /* Set gyro to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x17);
    thread_sleep_for(100);
    /* Set mag to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x1B);
    thread_sleep_for(100);
}

void IMU_BMX160::wakeUp()
{
    softReset();
    thread_sleep_for(100);
    setMagnConf();
    thread_sleep_for(100);
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    thread_sleep_for(100);
    /* Set gyro to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    thread_sleep_for(100);
    /* Set mag to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
    thread_sleep_for(100);
}

bool IMU_BMX160::softReset()
{
    int8_t rslt = BMX160_OK;
    if (Obmx160 == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    rslt = _softReset(Obmx160);
    if (rslt == 0)
        return true;
    else
        return false;
}

int8_t IMU_BMX160::_softReset(sBmx160Dev_t *dev)
{
    int8_t rslt = BMX160_OK;
    uint8_t data = BMX160_SOFT_RESET_CMD;
    if (dev == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    writeBmxReg(BMX160_COMMAND_REG_ADDR, data);
    thread_sleep_for(BMX160_SOFT_RESET_DELAY_MS);
    if (rslt == BMX160_OK)
    {
        IMU_BMX160::defaultParamSettg(dev);
    }
    return rslt;
}

void IMU_BMX160::defaultParamSettg(sBmx160Dev_t *dev)
{
    // Initializing accel and gyro params with
    dev->gyroCfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
    dev->gyroCfg.odr = BMX160_GYRO_ODR_100HZ;
    dev->gyroCfg.power = BMX160_GYRO_SUSPEND_MODE;
    dev->gyroCfg.range = BMX160_GYRO_RANGE_2000_DPS;
    dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
    dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
    dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
    dev->accelCfg.range = BMX160_ACCEL_RANGE_2G;

    dev->prevMagnCfg = dev->magnCfg;
    dev->prevGyroCfg = dev->gyroCfg;
    dev->prevAccelCfg = dev->accelCfg;
}

void IMU_BMX160::setMagnConf()
{
    writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x80);
    thread_sleep_for(50);
    // Sleep mode
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x01);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4B);
    // REPXY regular preset
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x04);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);
    // REPZ regular preset
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x0E);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);

    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
    writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);
    writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x08);
    writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x03);
    thread_sleep_for(50);
}

void IMU_BMX160::setGyroRange(eGyroRange_t bits)
{
    switch (bits)
    {
    case eGyroRange_125DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_125DPS;
        break;
    case eGyroRange_250DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
        break;
    case eGyroRange_500DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_500DPS;
        break;
    case eGyroRange_1000DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_1000DPS;
        break;
    case eGyroRange_2000DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
        break;
    default:
        gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
        break;
    }
}

void IMU_BMX160::setAccelRange(eAccelRange_t bits)
{
    switch (bits)
    {
    case eAccelRange_2G:
        accelRange = BMX160_ACCEL_MG_LSB_2G * 10;
        break;
    case eAccelRange_4G:
        accelRange = BMX160_ACCEL_MG_LSB_4G * 10;
        break;
    case eAccelRange_8G:
        accelRange = BMX160_ACCEL_MG_LSB_8G * 10;
        break;
    case eAccelRange_16G:
        accelRange = BMX160_ACCEL_MG_LSB_16G * 10;
        break;
    default:
        accelRange = BMX160_ACCEL_MG_LSB_2G * 10;
        break;
    }
}

void IMU_BMX160::getAllData(sBmx160SensorData_t *magn, sBmx160SensorData_t *gyro, sBmx160SensorData_t *accel)
{

    uint8_t data[23] = {0};
    int16_t x = 0, y = 0, z = 0;
    // put your main code here, to run repeatedly:
    readReg(BMX160_MAG_DATA_ADDR, data, 23);
    if (magn)
    {
        x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
        y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
        z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
        magn->x = x * BMX160_MAGN_UT_LSB;
        magn->y = y * BMX160_MAGN_UT_LSB;
        magn->z = z * BMX160_MAGN_UT_LSB;
    }
    if (gyro)
    {
        x = (int16_t)(((uint16_t)data[9] << 8) | data[8]);
        y = (int16_t)(((uint16_t)data[11] << 8) | data[10]);
        z = (int16_t)(((uint16_t)data[13] << 8) | data[12]);
        gyro->x = x * gyroRange;
        gyro->y = y * gyroRange;
        gyro->z = z * gyroRange;
    }
    if (accel)
    {
        x = (int16_t)(((uint16_t)data[15] << 8) | data[14]);
        y = (int16_t)(((uint16_t)data[17] << 8) | data[16]);
        z = (int16_t)(((uint16_t)data[19] << 8) | data[18]);
        accel->x = x * accelRange;
        accel->y = y * accelRange;
        accel->z = z * accelRange;
    }
}

void IMU_BMX160::writeBmxReg(uint8_t reg, uint8_t value)
{
    uint8_t buffer[1] = {value};
    writeReg(reg, buffer, 1);
}

// Implementation of following 3 functions taken from
// https://forums.mbed.com/t/converting-arduino-functions-to-mbed/9611/2

void IMU_BMX160::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    _pWire->lock();
    _pWire->start();
    _pWire->write(_addr << 1);
    _pWire->write(reg);
    for (uint16_t i = 0; i < len; i++)
        _pWire->write(pBuf[i]);
    _pWire->stop();
    _pWire->unlock();
}

void IMU_BMX160::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    _pWire->lock();
    _pWire->start();
    _pWire->write(_addr << 1 | 1);

    int writeResult = _pWire->write(reg);
    _pWire->stop();
    _pWire->unlock();

    if (writeResult != 1)
    {
        return;
    }
    _pWire->read(_addr << 1 | 1);
}

bool IMU_BMX160::scan()
{
    _pWire->lock();
    _pWire->start();

    int writeResult = _pWire->write(_addr << 1 | 1);
    _pWire->stop();
    _pWire->unlock();

    if (writeResult == 1)
    {
        return true;
    }
    return false;
}