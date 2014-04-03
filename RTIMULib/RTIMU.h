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

#ifndef _RTIMU_H
#define	_RTIMU_H

#include "RTMath.h"
#include "RTKalman.h"
#include "RTIMUHal.h"
#include "RTIMUSettings.h"

class RTIMU : public RTIMUHal
{
public:
    RTIMU(int kalmanType);
    virtual ~RTIMU();

    //  call the following to reset the Kalman

    void resetKalman() { m_kalman->reset(); }

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_kalman->setGyroEnable(enable);}
    void setAccelEnable(bool enable) { m_kalman->setAccelEnable(enable);}
    void setCompassEnable(bool enable) { m_kalman->setCompassEnable(enable);}

    //  call the following to enable debug messages

    void setDebugEnable(bool enable) { m_kalman->setDebugEnable(enable); }

    //  the following two functions get access to the measured pose (accel and compass)

    const RTVector3& getMeasuredPose() { return m_kalman->getMeasuredPose(); }
    const RTQuaternion& getMeasuredQPose() { return m_kalman->getMeasuredQPose(); }

    //  the following two functions get access to the computed Kalman pose

    const RTVector3& getKalmanPose() { return m_kalman->getKalmanPose(); }
    const RTQuaternion& getKalmanQPose() { return m_kalman->getKalmanQPose(); }

    //  setCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setCalibrationMode(bool enable) { m_calibrationMode = enable; }

    //  setCalibrationData configured the cal data and also enables use if valid

    void setCalibrationData(const bool valid, const RTVector3& compassMin, const RTVector3& compassMax);

    //  getCalibrationValid() returns true if the calibration data is being used

    bool getCalibrationValid() { return !m_calibrationMode && m_calibrationValid; }

    virtual bool IMUInit(RTIMUSettings *settings) = 0;
    virtual bool IMURead() = 0;

    const RTVector3& getGyro() { return m_gyroData; }       // gets gyro rates in radians/sec
    const RTVector3& getAccel() { return m_accelData; }     // get accel data in gs
    const RTVector3& getCompass() { return m_compassData; } // gets compass data in uT

protected:
    void updateKalman(uint64_t timestamp);                  // timestamp is in microseconds since epoch

    bool m_calibrationMode;                                 // true if cal mode so don't use cal data!
    bool m_calibrationValid;                                // tru if cal data is valid and can be used

    RTVector3 m_gyroData;                                   // the scaled gyro data
    RTVector3 m_accelData;                                  // the scaled accel data
    RTVector3 m_compassData;                                // the compass data

    RTKalman *m_kalman;
    bool m_firstKalmanUpdate;                               // microseconds since epoch of previous update
    uint64_t m_lastKalmanTime;

    float m_compassCalOffset[3];
    float m_compassCalScale[3];
 };

#endif // _RTIMU_H
