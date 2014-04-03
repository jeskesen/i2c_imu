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

#include "RTKalman4.h"

//  The QVALUE affects the gyro response.

#define KALMAN_QVALUE	0.001f

//  The RVALUE controls the influence of the accels and compass.
//  The bigger the value, the more sluggish the response.

#define KALMAN_RVALUE	0.0005f

RTKalman4::RTKalman4()
{
    m_Rk.fill(0);
    m_Q.fill(0);

    // initialize process noise covariance matrix

    for (int i = 0; i < KALMAN_STATE_LENGTH; i++)
        for (int j = 0; j < KALMAN_STATE_LENGTH; j++)
            m_Q.setVal(i, i, KALMAN_QVALUE);

    // initialize observation noise covariance matrix


    for (int i = 0; i < KALMAN_STATE_LENGTH; i++)
        for (int j = 0; j < KALMAN_STATE_LENGTH; j++)
            m_Rk.setVal(i, i, KALMAN_RVALUE);
}

RTKalman4::~RTKalman4()
{
}

void RTKalman4::reset()
{
    m_firstTime = true;
    m_kalmanPose = RTVector3();
    m_measuredPose = RTVector3();
}




void RTKalman4::predict()
{
    RTMatrix4x4 mat;
    RTQuaternion tQuat;
    RTFLOAT x2, y2, z2;

    //  compute the state transition matrix

    x2 = m_gyro.x() / (RTFLOAT)2.0;
    y2 = m_gyro.y() / (RTFLOAT)2.0;
    z2 = m_gyro.z() / (RTFLOAT)2.0;

    m_Fk.setVal(0, 1, -x2);
    m_Fk.setVal(0, 2, -y2);
    m_Fk.setVal(0, 3, -z2);

    m_Fk.setVal(1, 0, x2);
    m_Fk.setVal(1, 2, z2);
    m_Fk.setVal(1, 3, -y2);

    m_Fk.setVal(2, 0, y2);
    m_Fk.setVal(2, 1, -z2);
    m_Fk.setVal(2, 3, x2);

    m_Fk.setVal(3, 0, z2);
    m_Fk.setVal(3, 1, y2);
    m_Fk.setVal(3, 2, -x2);

    m_FkTranspose = m_Fk.transposed();

	// Predict new state estimate Xkk_1 = Fk * Xk_1k_1

    tQuat = m_Fk * m_stateQ;
    tQuat *= m_timeDelta;
    m_stateQ += tQuat;

//    m_stateQ.normalize();

    // Compute PDot = Fk * Pk_1k_1 + Pk_1k_1 * FkTranspose (note Pkk == Pk_1k_1 at this stage)

    m_PDot = m_Fk * m_Pkk;
    mat = m_Pkk * m_FkTranspose;
    m_PDot += mat;

    // add in Q to get the new prediction

    m_Pkk_1 = m_PDot + m_Q;

    //  multiply by deltaTime (variable name is now misleading though)

    m_Pkk_1 *= m_timeDelta;
}


void RTKalman4::update()
{
    RTQuaternion delta;
    RTMatrix4x4 Sk, SkInverse;

    if (m_enableCompass || m_enableAccel) {
        m_stateQError = m_measuredQPose - m_stateQ;
    } else {
        m_stateQError = RTQuaternion();
    }

    //	Compute residual covariance Sk = Hk * Pkk_1 * HkTranspose + Rk
    //  Note: since Hk is the identity matrix, this has been simplified

    Sk = m_Pkk_1 + m_Rk;

    //	Compute Kalman gain Kk = Pkk_1 * HkTranspose * SkInverse
    //  Note: again, the HkTranspose part is omitted

    SkInverse = Sk.inverted();

    m_Kk = m_Pkk_1 * SkInverse;

    if (m_debug)
        HAL_INFO(RTMath::display("Gain", m_Kk));

    // make new state estimate

    delta = m_Kk * m_stateQError;

    m_stateQ += delta;

    m_stateQ.normalize();

    //  produce new estimate covariance Pkk = (I - Kk * Hk) * Pkk_1
    //  Note: since Hk is the identity matrix, it is omitted

    m_Pkk.setToIdentity();
    m_Pkk -= m_Kk;
    m_Pkk = m_Pkk * m_Pkk_1;

    if (m_debug)
        HAL_INFO(RTMath::display("Cov", m_Pkk));
}

void RTKalman4::newIMUData(const RTVector3& accel, const RTVector3& gyro, const RTVector3& mag, RTFLOAT deltaTime)
{
    if (m_firstTime) {
        calculatePose(accel, mag);
        m_Fk.fill(0);

        //  init covariance matrix to something

        m_Pkk.fill(0);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m_Pkk.setVal(i,j, 0.5);

        // initialize the observation model Hk
        // Note: since the model is the state vector, this is an identity matrix so it won't be used

        //  initialize the poses

        m_stateQ.fromEuler(m_measuredPose);
        m_kalmanQPose = m_stateQ;
        m_kalmanPose = m_measuredPose;
        m_firstTime = false;
    } else {
        if (deltaTime == 0)
            return;

        if (m_debug) {
            HAL_INFO("\n------\n");
            HAL_INFO1("IMU update delta time: %f\n", deltaTime);
        }

        if (m_enableGyro)
            m_gyro = gyro;
        else
            m_gyro = RTVector3();

        m_timeDelta = deltaTime;

        calculatePose(accel, mag);

        predict();
        update();
        m_stateQ.toEuler(m_kalmanPose);
        m_kalmanQPose = m_stateQ;

        if (m_debug) {
            HAL_INFO(RTMath::displayRadians("Measured pose", m_measuredPose));
            HAL_INFO(RTMath::displayRadians("Kalman pose", m_kalmanPose));
            HAL_INFO(RTMath::displayRadians("Measured quat", m_measuredPose));
            HAL_INFO(RTMath::display("Kalman quat", m_stateQ));
            HAL_INFO(RTMath::display("Error quat", m_stateQError));
         }
    }
}
