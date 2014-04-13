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

	bool m_debug;
    bool m_enableGyro;                                      // enables gyro as input
    bool m_enableAccel;                                     // enables accel as input
    bool m_enableCompass;                                   // enables compass a input

    bool m_firstTime;                                       // if first time after reset
    uint64_t m_lastFusionTime;                              // for delta time calculation

    static const char *m_fusionNameMap[];                     // the fusion name array
};

#endif // _RTFUSION_H
