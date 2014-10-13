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

#ifndef _RTFUSION_H
#define	_RTFUSION_H

#include "RTIMULibDefs.h"

class RTFusion
{
public:

    RTFusion();
    virtual ~RTFusion();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_NULL; }

    //  reset() resets the fusion state but keeps any setting changes (such as enables)

    virtual void reset() {}

    //  newIMUData() should be called for subsequent updates
    //  the fusion fields are updated with the results

    virtual void newIMUData(RTIMU_DATA& /* data */) {}

    //  This static function returns performs the type to name mapping

    static const char *fusionName(int fusionType) { return m_fusionNameMap[fusionType]; }

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_enableGyro = enable; reset();}
    void setAccelEnable(bool enable) { m_enableAccel = enable; reset();}
    void setCompassEnable(bool enable) { m_enableCompass = enable; reset();}

    inline const RTVector3& getMeasuredPose() {return m_measuredPose;}
    inline const RTQuaternion& getMeasuredQPose() {return m_measuredQPose;}

    //  getAccelResiduals() gets the residual after subtracting gravity

    RTVector3 getAccelResiduals();

    void setDebugEnable(bool enable) { m_debug = enable; }

protected:
    void calculatePose(const RTVector3& accel, const RTVector3& mag); // generates pose from accels and heading

    RTVector3 m_gyro;                                       // current gyro sample
    RTVector3 m_accel;                                      // current accel sample
    RTVector3 m_compass;                                    // current compass sample

    RTQuaternion m_measuredQPose;       					// quaternion form of pose from measurement
    RTVector3 m_measuredPose;								// vector form of pose from measurement
    RTQuaternion m_fusionQPose;                             // quaternion form of pose from fusion
    RTVector3 m_fusionPose;                                 // vector form of pose from fusion

    RTQuaternion m_gravity;                                 // the gravity vector as a quaternion

    bool m_debug;
    bool m_enableGyro;                                      // enables gyro as input
    bool m_enableAccel;                                     // enables accel as input
    bool m_enableCompass;                                   // enables compass a input

    bool m_firstTime;                                       // if first time after reset
    uint64_t m_lastFusionTime;                              // for delta time calculation

    static const char *m_fusionNameMap[];                   // the fusion name array
};

#endif // _RTFUSION_H
