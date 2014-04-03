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

#ifndef _RTKALMAN_H
#define	_RTKALMAN_H

#include "RTMath.h"

class RTKalman
{
public:

    RTKalman();
    virtual ~RTKalman();

    //  reset() resets the kalman state but keeps any setting changes (such as enables)

    virtual void reset() {}

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    virtual void newIMUData(const RTVector3& accel, const RTVector3& gyro, const RTVector3& mag, float deltaTime);

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_enableGyro = enable; reset();}
    void setAccelEnable(bool enable) { m_enableAccel = enable; reset();}
    void setCompassEnable(bool enable) { m_enableCompass = enable; reset();}


    inline const RTVector3& getMeasuredPose() {return m_measuredPose;}
    inline const RTQuaternion& getMeasuredQPose() {return m_measuredQPose;}
    inline const RTVector3& getKalmanPose() {return m_kalmanPose;}
    inline const RTQuaternion& getKalmanQPose() {return m_kalmanQPose;}

    void setDebugEnable(bool enable) { m_debug = enable; }

protected:
    void calculatePose(const RTVector3& accel, const RTVector3& mag); // generates pose from accels and heading

    RTQuaternion m_kalmanQPose;                             // the computed pose from the quaternion state
    RTVector3 m_kalmanPose;                                 // the vector form of the pose
    RTQuaternion m_measuredQPose;       					// quaternion form of pose from measurement
    RTVector3 m_measuredPose;								// vector form of pose from measurement

	bool m_debug;
    bool m_enableGyro;                                      // enables gyro as input
    bool m_enableAccel;                                     // enables accel as input
    bool m_enableCompass;                                   // enables compass a input

    bool m_firstTime;                                       // if first time after reset
};

#endif // _RTKALMAN_H
