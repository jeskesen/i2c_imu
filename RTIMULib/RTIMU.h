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

#ifndef _RTIMU_H
#define	_RTIMU_H

#include "RTMath.h"
#include "RTFusion.h"
#include "RTIMULibDefs.h"
#include "RTIMUSettings.h"

class RTIMU
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

    //  setCompassCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setCompassCalibrationMode(bool enable) { m_compassCalibrationMode = enable; }

    //  setAccelCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setAccelCalibrationMode(bool enable) { m_accelCalibrationMode = enable; }

    //  setCalibrationData configures the cal data from settings and also enables use if valid

    void setCalibrationData();

    //  getCompassCalibrationValid() returns true if the compass min/max calibration data is being used

    bool getCompassCalibrationValid() { return !m_compassCalibrationMode && m_settings->m_compassCalValid; }

    //  getCompassCalibrationEllipsoidValid() returns true if the compass ellipsoid calibration data is being used

    bool getCompassCalibrationEllipsoidValid() { return !m_compassCalibrationMode && m_settings->m_compassCalEllipsoidValid; }

    //  getAccelCalibrationValid() returns true if the accel calibration data is being used

    bool getAccelCalibrationValid() { return !m_accelCalibrationMode && m_settings->m_accelCalValid; }

    const RTVector3& getGyro() { return m_imuData.gyro; }   // gets gyro rates in radians/sec
    const RTVector3& getAccel() { return m_imuData.accel; } // get accel data in gs
    const RTVector3& getCompass() { return m_imuData.compass; } // gets compass data in uT

    RTVector3 getAccelResiduals() { return m_fusion->getAccelResiduals(); }

protected:
    void gyroBiasInit();                                    // sets up gyro bias calculation
    void handleGyroBias();                                  // adjust gyro for bias
    void calibrateAverageCompass();                         // calibrate and smooth compass
    void calibrateAccel();                                  // calibrate the accelerometers
    void updateFusion();                                    // call when new data to update fusion state

    bool m_compassCalibrationMode;                          // true if cal mode so don't use cal data!
    bool m_accelCalibrationMode;                            // true if cal mode so don't use cal data!

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
