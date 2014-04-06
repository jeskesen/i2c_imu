//
//  Copyright (c) 2014 richards-tech
//
//  This file is part of RTIMULib
//
//  RTIMULib is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  RTIMULib is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with RTIMULib.  If not, see <http://www.gnu.org/licenses/>.
//

#include "RTIMUMPU9150.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMUMPU9150::RTIMUMPU9150(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUMPU9150::~RTIMUMPU9150()
{
}

bool RTIMUMPU9150::setLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9150_LPF_256:
    case MPU9150_LPF_188:
    case MPU9150_LPF_98:
    case MPU9150_LPF_42:
    case MPU9150_LPF_20:
    case MPU9150_LPF_10:
    case MPU9150_LPF_5:
        m_lpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9150 lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUMPU9150::setSampleRate(int rate)
{
    if ((rate < MPU9150_SAMPLERATE_MIN) || (rate > MPU9150_SAMPLERATE_MAX)) {
        HAL_ERROR1("Illegal sample rate %d\n", rate);
        return false;
    }
    m_sampleRate = rate;
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUMPU9150::setCompassRate(int rate)
{
    if ((rate < MPU9150_COMPASSRATE_MIN) || (rate > MPU9150_COMPASSRATE_MAX)) {
        HAL_ERROR1("Illegal compass rate %d\n", rate);
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9150::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9150_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9150_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9150_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9150_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        HAL_ERROR1("Illegal MPU9150 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUMPU9150::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9150_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case MPU9150_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case MPU9150_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case MPU9150_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9150 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUMPU9150::IMUInit()
{
    unsigned char result;
    int loop;
    unsigned char asa[3];

    m_firstTime = true;

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    m_bus = m_settings->m_I2CBus;

    setSampleRate(m_settings->m_MPU9150GyroAccelSampleRate);
    setCompassRate(m_settings->m_MPU9150CompassSampleRate);
    setLpf(m_settings->m_MPU9150GyroAccelLpf);
    setGyroFsr(m_settings->m_MPU9150GyroFsr);
    setAccelFsr(m_settings->m_MPU9150AccelFsr);

    setCalibrationData(m_settings->m_compassCalValid, m_settings->m_compassCalMin,
                              m_settings->m_compassCalMax);


    //  enable the I2C bus
    setI2CBus(m_bus);
    if (!I2COpen())
        return false;

    //  reset the MPU9150

    if (!I2CWrite(m_slaveAddr, MPU9150_PWR_MGMT_1, 0x80, "Failed to initiate MPU9150 reset"))
        return false;

    for (loop = 0; loop < 10; loop++) {
        if (!I2CRead(m_slaveAddr, MPU9150_PWR_MGMT_1, 1, &result,
                     "Failed to read back MPU9150 power management register"))
            return false;

        if ((result & 0x80) == 0)
            break;                                          // reset complete
        delayMs(50);
    }
    if (loop == 10) {
        HAL_ERROR("MPU9150 never came out of reset\n");
        return false;
    }

    if (!I2CWrite(m_slaveAddr, MPU9150_PWR_MGMT_1, 0x00, "Failed to stop MPU9150 reset"))
        return false;

    if (!I2CRead(m_slaveAddr, MPU9150_WHO_AM_I, 1, &result, "Failed to read MPU9150 id"))
        return false;

    if (result != 0x68) {
        HAL_ERROR1("Incorrect MPU9150 id %d\n", result);
        return false;
    }

    //  now configure the various components

    if (!I2CWrite(m_slaveAddr, MPU9150_LPF_CONFIG, m_lpf, "Failed to set lpf"))
        return false;

    if (!setSampleRate())
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_GYRO_CONFIG, m_gyroFsr, "Failed to set gyro fsr"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_ACCEL_CONFIG, m_accelFsr, "Failed to set accel fsr"))
         return false;

    //  now configure compass

    bypassOn();

    // get fuse ROM data

    if (!I2CWrite(AK8975_ADDRESS, AK8975_CNTL, 0, "Failed to set compass in power down mode 1")) {
        bypassOff();
        return false;
    }

    if (!I2CWrite(AK8975_ADDRESS, AK8975_CNTL, 0x0f, "Failed to set compass in fuse ROM mode")) {
        bypassOff();
        return false;
    }

    if (!I2CRead(AK8975_ADDRESS, AK8975_ASAX, 3, asa, "Failed to read compass fuse ROM")) {
        bypassOff();
        return false;
    }

    //  convert asa to usable scale factor

    m_compassAdjust[0] = ((float)asa[0] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[1] = ((float)asa[1] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[2] = ((float)asa[2] - 128.0) / 256.0 + 1.0f;

    if (!I2CWrite(AK8975_ADDRESS, AK8975_CNTL, 0, "Failed to set compass in power down mode 2")) {
        bypassOff();
        return false;
    }

    bypassOff();

    //  now set up MPU9150 to talk to the compass chip

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_MST_CTRL, 0x40, "Failed to set I2C master mode"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV0_ADDR, 0x80 | AK8975_ADDRESS, "Failed to set slave 0 address"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV0_REG, AK8975_ST1, "Failed to set slave 0 reg"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV0_CTRL, 0x88, "Failed to set slave 0 ctrl"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV1_ADDR, AK8975_ADDRESS, "Failed to set slave 1 address"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV1_REG, AK8975_CNTL, "Failed to set slave 1 reg"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV1_CTRL, 0x81, "Failed to set slave 1 ctrl"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV1_DO, 0x1, "Failed to set slave 1 DO"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_MST_DELAY_CTRL, 0x3, "Failed to set mst delay"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_YG_OFFS_TC, 0x80, "Failed to set yg offs tc"))
        return false;

    if (!setCompassRate())
        return false;

    //  enable the sensors

    if (!I2CWrite(m_slaveAddr, MPU9150_PWR_MGMT_1, 1, "Failed to set pwr_mgmt_1"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_PWR_MGMT_2, 0, "Failed to set pwr_mgmt_2"))
         return false;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return false;

    m_gyroAlpha = 1.0f / m_sampleRate;
    m_gyroStartTime = RTMath::currentUSecsSinceEpoch();
    m_gyroLearning = true;

    HAL_INFO("MPU9150 init complete\n");
    return true;
}

bool RTIMUMPU9150::resetFifo()
{
    if (!I2CWrite(m_slaveAddr, MPU9150_INT_ENABLE, 0, "Writing int enable"))
        return false;
    if (!I2CWrite(m_slaveAddr, MPU9150_FIFO_EN, 0, "Writing fifo enable"))
        return false;
    if (!I2CWrite(m_slaveAddr, MPU9150_USER_CTRL, 0, "Writing user control"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_USER_CTRL, 0x04, "Resetting fifo"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_USER_CTRL, 0x60, "Enabling the fifo"))
        return false;

    delayMs(50);

    if (!I2CWrite(m_slaveAddr, MPU9150_INT_ENABLE, 1, "Writing int enable"))
        return false;

    if (!I2CWrite(m_slaveAddr, MPU9150_FIFO_EN, 0x78, "Failed to set FIFO enables"))
        return false;

    return true;
}

bool RTIMUMPU9150::bypassOn()
{
    unsigned char userControl;

    if (!I2CRead(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    if (!I2CWrite(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    delayMs(50);

    if (!I2CWrite(m_slaveAddr, MPU9150_INT_PIN_CFG, 0x82, "Failed to write int_pin_cfg reg"))
        return false;

    delayMs(50);
    return true;
}


bool RTIMUMPU9150::bypassOff()
{
    unsigned char userControl;

    if (!I2CRead(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!I2CWrite(m_slaveAddr, MPU9150_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    delayMs(50);

    if (!I2CWrite(m_slaveAddr, MPU9150_INT_PIN_CFG, 0x80, "Failed to write int_pin_cfg reg"))
         return false;

    delayMs(50);
    return true;
}

bool RTIMUMPU9150::setSampleRate()
{
    int clockRate = 1000;

    if (m_lpf == MPU9150_LPF_256)
        clockRate = 8000;

    if (!I2CWrite(m_slaveAddr, MPU9150_SMPRT_DIV, (unsigned char)(clockRate / m_sampleRate - 1),
                  "Failed to set sample rate"))
        return false;

    return true;
}

bool RTIMUMPU9150::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    if (!I2CWrite(m_slaveAddr, MPU9150_I2C_SLV4_CTRL, rate, "Failed to set slave ctrl 4"))
         return false;
    return true;
}

int RTIMUMPU9150::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUMPU9150::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!I2CRead(m_slaveAddr, MPU9150_FIFO_COUNT_H, 2, fifoCount, "Failed to read fifo count"))
         return false;

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    if (count == 1024) {
        HAL_INFO("MPU9150 fifo has overflowed");
        resetFifo();
        return false;
    }

    if (count > 480) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= 120) {
            if (!I2CRead(m_slaveAddr, MPU9150_FIFO_R_W, 12, fifoData, "Failed to read fifo data"))
                return false;
            count -= 12;
            m_imuData.timestamp += m_sampleInterval;
        }
    }

    if (count < 12)
        return false;

    if (!I2CRead(m_slaveAddr, MPU9150_FIFO_R_W, 12, fifoData, "Failed to read fifo data"))
        return false;

    if (!I2CRead(m_slaveAddr, MPU9150_EXT_SENS_DATA_00, 8, compassData, "Failed to read compass data"))
        return false;

    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.3f, false);

    if (m_gyroLearning) {
        // update gyro bias

        m_gyroBias.setX((1.0 - m_gyroAlpha) * m_gyroBias.x() + m_gyroAlpha * m_imuData.gyro.x());
        m_gyroBias.setY((1.0 - m_gyroAlpha) * m_gyroBias.y() + m_gyroAlpha * m_imuData.gyro.y());
        m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_gyroBias.z() + m_gyroAlpha * m_imuData.gyro.z());

        if ((RTMath::currentUSecsSinceEpoch() - m_gyroStartTime) > 5000000)
            m_gyroLearning = false;                     // only do this for 5 seconds
    }

    //  sort out gyro axes and correct for bias

    m_imuData.gyro.setX(m_imuData.gyro.x() - m_gyroBias.x());
    m_imuData.gyro.setY(-(m_imuData.gyro.y() - m_gyroBias.y()));
    m_imuData.gyro.setZ(-(m_imuData.gyro.z() - m_gyroBias.z()));

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes

    float temp;

    temp = m_imuData.compass.x();
    m_imuData.compass.setX(m_imuData.compass.y());
    m_imuData.compass.setY(-temp);

    //  use the fuse data adjustments

    m_imuData.compass.setX(m_imuData.compass.x() * m_compassAdjust[0]);
    m_imuData.compass.setY(m_imuData.compass.y() * m_compassAdjust[1]);
    m_imuData.compass.setZ(m_imuData.compass.z() * m_compassAdjust[2]);

    //  calibrate if required

    if (!m_calibrationMode && m_calibrationValid) {
        m_imuData.compass.setX((m_imuData.compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_imuData.compass.setY((m_imuData.compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_imuData.compass.setZ((m_imuData.compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);
    }

    //  update running average

    m_compassAverage.setX(m_imuData.compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_imuData.compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_imuData.compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));
    m_imuData.compass = m_compassAverage;

    if (m_firstTime)
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    else
        m_imuData.timestamp += m_sampleInterval;

    m_firstTime = false;

    //  now update the filter

    updateFusion();

    return true;
}
