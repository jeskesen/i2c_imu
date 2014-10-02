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

#include "RTFusion.h"
#include "RTIMUHal.h"

const char *RTFusion::m_fusionNameMap[] = {
    "NULL",
    "Kalman STATE4",
    "RTQF"};

RTFusion::RTFusion()
{
    m_debug = false;
    m_firstTime = true;
    m_enableGyro = true;
    m_enableAccel = true;
    m_enableCompass = true;
}

RTFusion::~RTFusion()
{
}

void RTFusion::calculatePose(const RTVector3& accel, const RTVector3& mag)
{
    RTQuaternion m;
    RTQuaternion q;

    if (m_enableAccel) {
        accel.accelToEuler(m_measuredPose);
    } else {
        m_measuredPose = m_fusionPose;
        m_measuredPose.setZ(0);
    }

    if (m_enableCompass) {
        q.fromEuler(m_measuredPose);
        m.setScalar(0);
        m.setX(mag.x());
        m.setY(mag.y());
        m.setZ(mag.z());

        m = q * m * q.conjugate();
        m_measuredPose.setZ(-atan2(m.y(), m.x()));
    } else {
        m_measuredPose.setZ(m_fusionPose.z());
    }

    m_measuredQPose.fromEuler(m_measuredPose);

    //  check for quaternion aliasing. If the quaternion has the wrong sign
    //  the kalman filter will be very unhappy.

    int maxIndex = -1;
    RTFLOAT maxVal = -1000;

    for (int i = 0; i < 4; i++) {
        if (fabs(m_measuredQPose.data(i)) > maxVal) {
            maxVal = fabs(m_measuredQPose.data(i));
            maxIndex = i;
        }
    }

    //  if the biggest component has a different sign in the measured and kalman poses,
    //  change the sign of the measured pose to match.

    if (((m_measuredQPose.data(maxIndex) < 0) && (m_fusionQPose.data(maxIndex) > 0)) ||
            ((m_measuredQPose.data(maxIndex) > 0) && (m_fusionQPose.data(maxIndex) < 0))) {
        m_measuredQPose.setScalar(-m_measuredQPose.scalar());
        m_measuredQPose.setX(-m_measuredQPose.x());
        m_measuredQPose.setY(-m_measuredQPose.y());
        m_measuredQPose.setZ(-m_measuredQPose.z());
        m_measuredQPose.toEuler(m_measuredPose);
    }
}

