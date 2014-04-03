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

#ifndef _RTKALMAN4_H
#define	_RTKALMAN4_H

#include "RTKalman.h"

#define KALMAN_QUATERNION_LENGTH	4

#define	KALMAN_STATE_LENGTH	4								// just the quaternion for the moment

class RTKalman4 : public RTKalman
{
public:
    RTKalman4();
    ~RTKalman4();

    //  reset() resets the kalman state but keeps any setting changes (such as enables)

    void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(const RTVector3& accel, const RTVector3& gyro, const RTVector3& mag, RTFLOAT deltaTime);

    //  the following two functions can be called before initKalman to customize the covariance matrices

    void setQMatrix(RTMatrix4x4 Q) {  m_Q = Q; reset();}
    void setRkMatrix(RTMatrix4x4 Rk) { m_Rk = Rk; reset();}

private:
	void predict();
	void update();

    RTVector3 m_gyro;										// unbiased gyro data
    RTFLOAT m_timeDelta;                                    // time between predictions

    RTQuaternion m_stateQ;									// quaternion state vector
    RTQuaternion m_stateQError;                             // difference between stateQ and measuredQ

    RTMatrix4x4 m_Kk;                                       // the Kalman gain matrix
    RTMatrix4x4 m_Pkk_1;                                    // the predicted estimated covariance matrix
    RTMatrix4x4 m_Pkk;                                      // the updated estimated covariance matrix
    RTMatrix4x4 m_PDot;                                     // the derivative of the covariance matrix
    RTMatrix4x4 m_Q;                                        // process noise covariance
    RTMatrix4x4 m_Fk;                                       // the state transition matrix
    RTMatrix4x4 m_FkTranspose;                              // the state transition matrix transposed
    RTMatrix4x4 m_Rk;                                       // the measurement noise covariance

    //  Note: SInce Hk ends up being the identity matrix, these are omitted

//    RTMatrix4x4 m_Hk;                                     // map from state to measurement
//    RTMatrix4x4> m_HkTranspose;                           // transpose of map
};

#endif // _RTKALMAN4_H
