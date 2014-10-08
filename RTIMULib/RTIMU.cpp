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
#include "RTIMUSettings.h"

#include "RTIMUNull.h"
#include "RTIMUMPU9150.h"
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

    case RTIMU_TYPE_AUTODISCOVER:
        if (settings->discoverIMU(settings->m_imuType, settings->m_I2CSlaveAddress)) {
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

    m_calibrationMode = false;
    m_calibrationValid = false;

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

void RTIMU::setCalibrationData(const bool valid,
                               const RTVector3& compassMin, const RTVector3& compassMax)
{
    float maxDelta = -1;
    float delta;

    m_calibrationValid = valid;
    if (!valid) {
        HAL_INFO("Compass not calibrated\n");
        return;
    }

    //  find biggest range

    for (int i = 0; i < 3; i++) {
        if ((compassMax.data(i) - compassMin.data(i)) > maxDelta)
            maxDelta = compassMax.data(i) - compassMin.data(i);
    }
    if (maxDelta < 0) {
        HAL_ERROR("Error in compass calibration data\n");
        return;
    }
    maxDelta /= 2.0f;                                       // this is the max +/- range

    for (int i = 0; i < 3; i++) {
        delta = (compassMax.data(i) - compassMin.data(i)) / 2.0f;
        m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
        m_compassCalOffset[i] = (compassMax.data(i) + compassMin.data(i)) / 2.0f;
    }
    m_calibrationValid = true;
    HAL_INFO("Compass is calibrated\n");
}

void RTIMU::gyroBiasInit()
{
    m_gyroAlpha = 2.0f / m_sampleRate;
    m_gyroSampleCount = 0;
}

void RTIMU::handleGyroBias()
{
    RTVector3 deltaAccel = m_previousAccel;
    deltaAccel -= m_imuData.accel;   // compute difference
    m_previousAccel = m_imuData.accel;

    if ((deltaAccel.length() < RTIMU_FUZZY_ACCEL_ZERO) && (m_imuData.gyro.length() < RTIMU_FUZZY_GYRO_ZERO)) {
        // what we are seeing on the gyros should be bias only so learn from this
        m_settings->m_gyroBias.setX((1.0 - m_gyroAlpha) * m_settings->m_gyroBias.x() + m_gyroAlpha * m_imuData.gyro.x());
        m_settings->m_gyroBias.setY((1.0 - m_gyroAlpha) * m_settings->m_gyroBias.y() + m_gyroAlpha * m_imuData.gyro.y());
        m_settings->m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_settings->m_gyroBias.z() + m_gyroAlpha * m_imuData.gyro.z());

        if (m_gyroSampleCount < (5 * m_sampleRate)) {
            m_gyroSampleCount++;

            if (m_gyroSampleCount == (5 * m_sampleRate)) {
                // this could have been true already of course
                m_settings->m_gyroBiasValid = true;
                m_settings->saveSettings();
            }
        }
    }

    m_imuData.gyro -= m_settings->m_gyroBias;
}

void RTIMU::calibrateAverageCompass()
{
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
}

void RTIMU::updateFusion()
{
    m_fusion->newIMUData(m_imuData);
}

bool RTIMU::IMUGyroBiasValid()
{
    return m_settings->m_gyroBiasValid;
}


