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
#include "RTKalman.h"

#include "RTKalman4.h"

RTIMU::RTIMU(int kalmanType)
{
    m_firstKalmanUpdate = true;
    m_calibrationMode = false;
    m_calibrationValid = false;

    switch (kalmanType) {
    case RTKALMAN_TYPE_STATE4:
        m_kalman = new RTKalman4();
        HAL_INFO("Using Kalman STATE4\n");
        break;

    default:
        m_kalman = new RTKalman();
        HAL_INFO("Using Kalman NULL\n");
        break;
    }
}

RTIMU::~RTIMU()
{
    delete m_kalman;
    m_kalman = NULL;
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

void RTIMU::updateKalman(uint64_t timestamp)
{
    m_kalman->newIMUData(m_accelData, m_gyroData, m_compassData, (RTFLOAT)(timestamp - m_lastKalmanTime) / (RTFLOAT)1000000.0);
    m_lastKalmanTime = timestamp;
}

