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
#include "RTIMUHal.h"
#include "RTFusion.h"
#include "RTIMULibDefs.h"

class RTIMUSettings;

class RTIMU : public RTIMUHal
{
public:
    //  IMUs should always be created with the following call

    static RTIMU *createIMU(RTIMUSettings *settings);

    //  Constructor/destructor

    RTIMU(RTIMUSettings *settings);
    virtual ~RTIMU();

    //  These functions must be provided by sub classes

    virtual const char *IMUName() = 0;                      // the name of the IMU
    virtual int IMUType() = 0;                              // the type code of the IMU
    virtual bool IMUInit() = 0;                             // set up the IMU
    virtual int IMUGetPollInterval() = 0;                   // returns the recommended poll interval in mS
    virtual bool IMURead() = 0;                             // get a sample

    //  This one wanted a similar name but isn't pure virtual

    virtual bool IMUCompassCalValid() { return m_calibrationValid; }

    // returns true if enough samples for valid data

    virtual bool IMUGyroBiasValid();

    //  call the following to reset the fusion algorithm

    void resetFusion() { m_fusion->reset(); }

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_fusion->setGyroEnable(enable);}
    void setAccelEnable(bool enable) { m_fusion->setAccelEnable(enable);}
    void setCompassEnable(bool enable) { m_fusion->setCompassEnable(enable);}

    //  call the following to enable debug messages

    void setDebugEnable(bool enable) { m_fusion->setDebugEnable(enable); }

    //  getIMUData returns the standard outputs of the IMU and fusion filter

    const RTIMU_DATA& getIMUData() { return m_imuData; }

    //  the following two functions get access to the measured pose (accel and compass)

    const RTVector3& getMeasuredPose() { return m_fusion->getMeasuredPose(); }
    const RTQuaternion& getMeasuredQPose() { return m_fusion->getMeasuredQPose(); }

    //  setCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setCalibrationMode(bool enable) { m_calibrationMode = enable; }

    //  setCalibrationData configured the cal data and also enables use if valid

    void setCalibrationData(const bool valid, const RTVector3& compassMin, const RTVector3& compassMax);

    //  getCalibrationValid() returns true if the calibration data is being used

    bool getCalibrationValid() { return !m_calibrationMode && m_calibrationValid; }

    const RTVector3& getGyro() { return m_imuData.gyro; }   // gets gyro rates in radians/sec
    const RTVector3& getAccel() { return m_imuData.accel; } // get accel data in gs
    const RTVector3& getCompass() { return m_imuData.compass; } // gets compass data in uT

protected:
    void gyroBiasInit();                                    // sets up gyro bias calculation
    void handleGyroBias();                                  // adjust gyro for bias
    void calibrateAverageCompass();                         // calibrate and smooth compass
    void updateFusion();                                    // call when new data to update fusion state

    bool m_calibrationMode;                                 // true if cal mode so don't use cal data!
    bool m_calibrationValid;                                // tru if call data is valid and can be used

    RTIMU_DATA m_imuData;                                   // the data from the IMU

    RTIMUSettings *m_settings;                              // the settings object pointer

    RTFusion *m_fusion;                                     // the fusion algorithm

    int m_sampleRate;                                       // samples per second
    uint64_t m_sampleInterval;                              // interval between samples in microseonds

    RTFLOAT m_gyroAlpha;                                    // gyro bias learning rate
    int m_gyroSampleCount;                                  // number of gyro samples used

    RTVector3 m_previousAccel;                              // previous step accel for gyro learning

    float m_compassCalOffset[3];
    float m_compassCalScale[3];
    RTVector3 m_compassAverage;                             // a running average to smooth the mag outputs

 };

#endif // _RTIMU_H
