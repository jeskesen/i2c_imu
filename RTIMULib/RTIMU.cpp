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

#include "RTIMU.h"
#include "RTFusionKalman4.h"
#include "RTIMUSettings.h"

#include "RTIMUNull.h"
#include "RTIMUMPU9150.h"
#include "RTIMUGD20HM303D.h"
#include "RTIMUGD20M303DLHC.h"

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
    switch (settings->m_imuType) {
    case RTIMU_TYPE_MPU9150:
        return new RTIMUMPU9150(settings);

    case RTIMU_TYPE_GD20HM303D:
        return new RTIMUGD20HM303D(settings);

    case RTIMU_TYPE_GD20M303DLHC:
        return new RTIMUGD20M303DLHC(settings);

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
        HAL_INFO("Using Kalman STATE4\n");
        break;

    default:
        m_fusion = new RTFusion();
        HAL_INFO("Using Kalman NULL\n");
        break;
    }
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

void RTIMU::updateFusion()
{
    m_fusion->newIMUData(m_imuData);
}

