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

#include "RTIMUGD20M303.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMUGD20M303::RTIMUGD20M303(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMUGD20M303::~RTIMUGD20M303()
{
}

bool RTIMUGD20M303::IMUInit()
{
    unsigned char result;

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

    m_gyroSlaveAddr = m_settings->m_I2CSlaveAddress;
    if (m_gyroSlaveAddr == L3GD20H_ADDRESS0)
        m_accelCompassSlaveAddr = LSM303D_ADDRESS0;
    else
        m_accelCompassSlaveAddr = LSM303D_ADDRESS1;

    m_bus = m_settings->m_I2CBus;

    setCalibrationData(m_settings->m_compassCalValid, m_settings->m_compassCalMin,
                              m_settings->m_compassCalMax);


    //  enable the I2C bus
    setI2CBus(m_bus);
    if (!I2COpen())
        return false;

    //  Set up the gyro

    if (!I2CRead(m_gyroSlaveAddr, L3GD20H_WHO_AM_I, 1, &result, "Failed to read L3GD20H id"))
        return false;

    if (result != 0xd7) {
        HAL_ERROR1("Incorrect L3GD20H id %d\n", result);
        return false;
    }

    if (!setGyroSampleRate())
            return false;

    if (!setGyroCTRL2())
            return false;

    if (!setGyroCTRL4())
            return false;

    //  Set up the accel/compass

    m_imuData.accel = RTVector3(0, 0, 1);

    if (!I2CRead(m_accelCompassSlaveAddr, LSM303D_WHO_AM_I, 1, &result, "Failed to read LSM303D id"))
        return false;

    if (result != 0x49) {
        HAL_ERROR1("Incorrect LSM303D id %d\n", result);
        return false;
    }



    m_gyroAlpha = 0.8f;
    m_gyroStartTime = RTMath::currentUSecsSinceEpoch();
    m_gyroLearning = true;

    HAL_INFO("GD20M303 init complete\n");
    return true;
}

bool RTIMUGD20M303::setGyroSampleRate()
{
    unsigned char ctrl1;
    unsigned char lowOdr = 0;

    switch (m_settings->m_GD20M303GyroSampleRate) {
    case L3GD20H_SAMPLERATE_12_5:
        ctrl1 = 0x0f;
        lowOdr = 1;
        m_sampleRate = 13;
        break;

    case L3GD20H_SAMPLERATE_25:
        ctrl1 = 0x4f;
        lowOdr = 1;
        m_sampleRate = 25;
        break;

    case L3GD20H_SAMPLERATE_50:
        ctrl1 = 0x8f;
        lowOdr = 1;
        m_sampleRate = 50;
        break;

    case L3GD20H_SAMPLERATE_100:
        ctrl1 = 0x0f;
        m_sampleRate = 100;
        break;

    case L3GD20H_SAMPLERATE_200:
        ctrl1 = 0x4f;
        m_sampleRate = 200;
        break;

    case L3GD20H_SAMPLERATE_400:
        ctrl1 = 0x8f;
        m_sampleRate = 400;
        break;

    case L3GD20H_SAMPLERATE_800:
        ctrl1 = 0xcf;
        m_sampleRate = 800;
        break;

    default:
        HAL_ERROR1("Illegal L3GD20H sample rate code %d", m_settings->m_GD20M303GyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_GD20M303GyroBW) {
    case L3GD20H_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case L3GD20H_BANDWIDTH_1:
        ctrl1 |= 0x10;
        break;

    case L3GD20H_BANDWIDTH_2:
        ctrl1 |= 0x20;
        break;

    case L3GD20H_BANDWIDTH_3:
        ctrl1 |= 0x30;
        break;

     }

    if (!I2CWrite(m_gyroSlaveAddr, L3GD20H_CTRL1, 0, "Failed to reset L3GD20"))
        return false;

    if (!I2CWrite(m_gyroSlaveAddr, L3GD20H_LOW_ODR, lowOdr, "Failed to set L3GD20 LOW_ODR"))
        return false;

    return (I2CWrite(m_gyroSlaveAddr, L3GD20H_CTRL1, ctrl1, "Failed to set L3GD20 CTRL1"));
}

bool RTIMUGD20M303::setGyroCTRL2()
{
    if ((m_settings->m_GD20M303GyroHpf < L3GD20H_HPF_0) || (m_settings->m_GD20M303GyroHpf > L3GD20H_HPF_9)) {
        HAL_ERROR1("Illegal L3GD20H high pass filter code %d\n", m_settings->m_GD20M303GyroHpf);
        return false;
    }
    return I2CWrite(m_gyroSlaveAddr,  L3GD20H_CTRL2, m_settings->m_GD20M303GyroHpf, "Failed to set L3GD20 CTRL2");
}

bool RTIMUGD20M303::setGyroCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_GD20M303GyroFsr) {
    case L3GD20H_FSR_245:
        ctrl4 = 0x00;
        m_gyroScale = (RTMATH_PI * 245) / (32768 * 180.0);
        break;

    case L3GD20H_FSR_500:
        ctrl4 = 0x10;
        m_gyroScale = (RTMATH_PI * 500) / (32768 * 180.0);
        break;

    case L3GD20H_FSR_2000:
        ctrl4 = 0x20;
        m_gyroScale = (RTMATH_PI * 2000) / (32768 * 180.0);
        break;

    default:
        HAL_ERROR1("Illegal L3GD20H FSR code %d\n", m_settings->m_GD20M303GyroFsr);
        return false;
    }
    return I2CWrite(m_gyroSlaveAddr,  L3GD20H_CTRL4, ctrl4, "Failed to set L3GD20 CTRL4");
}

int RTIMUGD20M303::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUGD20M303::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];

    if (!I2CRead(m_gyroSlaveAddr, L3GD20H_STATUS, 1, &status, "Failed to read L3GD20H status"))
        return false;

    if ((status & 0x8) == 0)
        return false;                                       // no sample available yet

    if (!I2CRead(m_gyroSlaveAddr, 0x80 | L3GD20H_OUT_X_L, 6, gyroData, "Failed to read L3GD20H data"))
        return false;

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);

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
