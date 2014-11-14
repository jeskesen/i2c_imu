////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMU.h"
#include "RTFusionKalman4.h"
#include "RTFusionRTQF.h"

#include "RTIMUNull.h"
#include "RTIMUMPU9150.h"
#include "RTIMUMPU9250.h"
#include "RTIMUGD20HM303D.h"
#include "RTIMUGD20M303DLHC.h"
#include "RTIMULSM9DS0.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_GYRO_ZERO      0.20

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_ACCEL_ZERO      0.05

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
    switch (settings->m_imuType) {
    case RTIMU_TYPE_MPU9150:
        return new RTIMUMPU9150(settings);

    case RTIMU_TYPE_GD20HM303D:
        return new RTIMUGD20HM303D(settings);

    case RTIMU_TYPE_GD20M303DLHC:
        return new RTIMUGD20M303DLHC(settings);

    case RTIMU_TYPE_LSM9DS0:
        return new RTIMULSM9DS0(settings);

    case RTIMU_TYPE_MPU9250:
        return new RTIMUMPU9250(settings);

    case RTIMU_TYPE_AUTODISCOVER:
        if (settings->discoverIMU(settings->m_imuType, settings->m_busIsI2C, settings->m_I2CSlaveAddress)) {
            settings->saveSettings();
            return RTIMU::createIMU(settings);
        }
        return NULL;

    case RTIMU_TYPE_NULL:
        return new RTIMUNull(settings);

    default:
        return NULL;
    }
}


RTIMU::RTIMU(RTIMUSettings *settings)
{
    m_settings = settings;

    m_compassCalibrationMode = false;
    m_accelCalibrationMode = false;

    switch (m_settings->m_fusionType) {
    case RTFUSION_TYPE_KALMANSTATE4:
        m_fusion = new RTFusionKalman4();
        break;

    case RTFUSION_TYPE_RTQF:
        m_fusion = new RTFusionRTQF();
        break;

    default:
        m_fusion = new RTFusion();
        break;
    }
    HAL_INFO1("Using fusion algorithm %s\n", RTFusion::fusionName(m_settings->m_fusionType));
}

RTIMU::~RTIMU()
{
    delete m_fusion;
    m_fusion = NULL;
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;

    if (m_settings->m_compassCalValid) {
        //  find biggest range

        for (int i = 0; i < 3; i++) {
            if ((m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) > maxDelta)
                maxDelta = m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i);
        }
        if (maxDelta < 0) {
            HAL_ERROR("Error in compass calibration data\n");
            return;
        }
        maxDelta /= 2.0f;                                       // this is the max +/- range

        for (int i = 0; i < 3; i++) {
            delta = (m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
            m_compassCalOffset[i] = (m_settings->m_compassCalMax.data(i) + m_settings->m_compassCalMin.data(i)) / 2.0f;
        }
    }

    if (m_settings->m_compassCalValid) {
        HAL_INFO("Using min/max compass calibration\n");
    } else {
        HAL_INFO("min/max compass calibration not in use\n");
    }

    if (m_settings->m_compassCalEllipsoidValid) {
        HAL_INFO("Using ellipsoid compass calibration\n");
    } else {
        HAL_INFO("Ellipsoid compass calibration not in use\n");
    }

    if (m_settings->m_accelCalValid) {
        HAL_INFO("Using accel calibration\n");
    } else {
        HAL_INFO("Accel calibration not in use\n");
    }
}

bool RTIMU::setGyroContinuousLearningAlpha(RTFLOAT alpha)
{
    if ((alpha < 0.0) || (alpha >= 1.0))
        return false;

    m_gyroContinuousAlpha = alpha;
    return true;
}


void RTIMU::gyroBiasInit()
{
    m_gyroLearningAlpha = 2.0f / m_sampleRate;
    m_gyroContinuousAlpha = 0.01f / m_sampleRate;
    m_gyroSampleCount = 0;
}

void RTIMU::handleGyroBias()
{
    RTVector3 deltaAccel = m_previousAccel;
    deltaAccel -= m_imuData.accel;   // compute difference
    m_previousAccel = m_imuData.accel;

    if ((deltaAccel.length() < RTIMU_FUZZY_ACCEL_ZERO) && (m_imuData.gyro.length() < RTIMU_FUZZY_GYRO_ZERO)) {
        // what we are seeing on the gyros should be bias only so learn from this

        if (m_gyroSampleCount < (5 * m_sampleRate)) {
            m_settings->m_gyroBias.setX((1.0 - m_gyroLearningAlpha) * m_settings->m_gyroBias.x() + m_gyroLearningAlpha * m_imuData.gyro.x());
            m_settings->m_gyroBias.setY((1.0 - m_gyroLearningAlpha) * m_settings->m_gyroBias.y() + m_gyroLearningAlpha * m_imuData.gyro.y());
            m_settings->m_gyroBias.setZ((1.0 - m_gyroLearningAlpha) * m_settings->m_gyroBias.z() + m_gyroLearningAlpha * m_imuData.gyro.z());

            m_gyroSampleCount++;

            if (m_gyroSampleCount == (5 * m_sampleRate)) {
                // this could have been true already of course
                m_settings->m_gyroBiasValid = true;
                m_settings->saveSettings();
            }
        } else {
            m_settings->m_gyroBias.setX((1.0 - m_gyroContinuousAlpha) * m_settings->m_gyroBias.x() + m_gyroContinuousAlpha * m_imuData.gyro.x());
            m_settings->m_gyroBias.setY((1.0 - m_gyroContinuousAlpha) * m_settings->m_gyroBias.y() + m_gyroContinuousAlpha * m_imuData.gyro.y());
            m_settings->m_gyroBias.setZ((1.0 - m_gyroContinuousAlpha) * m_settings->m_gyroBias.z() + m_gyroContinuousAlpha * m_imuData.gyro.z());
        }
    }

    m_imuData.gyro -= m_settings->m_gyroBias;
}

void RTIMU::calibrateAverageCompass()
{
    //  calibrate if required

    if (getCompassCalibrationValid()) {
        m_imuData.compass.setX((m_imuData.compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_imuData.compass.setY((m_imuData.compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_imuData.compass.setZ((m_imuData.compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);

        if (m_settings->m_compassCalEllipsoidValid) {
            RTVector3 ev = m_imuData.compass;
            ev -= m_settings->m_compassCalEllipsoidOffset;

            m_imuData.compass.setX(ev.x() * m_settings->m_compassCalEllipsoidCorr[0][0] +
                ev.y() * m_settings->m_compassCalEllipsoidCorr[0][1] +
                ev.z() * m_settings->m_compassCalEllipsoidCorr[0][2]);

            m_imuData.compass.setY(ev.x() * m_settings->m_compassCalEllipsoidCorr[1][0] +
                ev.y() * m_settings->m_compassCalEllipsoidCorr[1][1] +
                ev.z() * m_settings->m_compassCalEllipsoidCorr[1][2]);

            m_imuData.compass.setZ(ev.x() * m_settings->m_compassCalEllipsoidCorr[2][0] +
                ev.y() * m_settings->m_compassCalEllipsoidCorr[2][1] +
                ev.z() * m_settings->m_compassCalEllipsoidCorr[2][2]);
        }
    }

    //  update running average

    m_compassAverage.setX(m_imuData.compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_imuData.compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_imuData.compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));

    m_imuData.compass = m_compassAverage;
}

void RTIMU::calibrateAccel()
{
    if (!getAccelCalibrationValid())
        return;

    if (m_imuData.accel.x() >= 0)
        m_imuData.accel.setX(m_imuData.accel.x() / m_settings->m_accelCalMax.x());
    else
        m_imuData.accel.setX(m_imuData.accel.x() / -m_settings->m_accelCalMin.x());

    if (m_imuData.accel.y() >= 0)
        m_imuData.accel.setY(m_imuData.accel.y() / m_settings->m_accelCalMax.y());
    else
        m_imuData.accel.setY(m_imuData.accel.y() / -m_settings->m_accelCalMin.y());

    if (m_imuData.accel.z() >= 0)
        m_imuData.accel.setZ(m_imuData.accel.z() / m_settings->m_accelCalMax.z());
    else
        m_imuData.accel.setZ(m_imuData.accel.z() / -m_settings->m_accelCalMin.z());
}

void RTIMU::updateFusion()
{
    m_fusion->newIMUData(m_imuData);
}

bool RTIMU::IMUGyroBiasValid()
{
    return m_settings->m_gyroBiasValid;
}

