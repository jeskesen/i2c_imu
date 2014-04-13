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

#ifndef _RTFUSIONRTQF_H
#define	_RTFUSIONRTQF_H

#include "RTFusion.h"

class RTFusionRTQF : public RTFusion
{
public:
    RTFusionRTQF();
    ~RTFusionRTQF();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_RTQF; }

    //  reset() resets the state but keeps any setting changes (such as enables)

    void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(RTIMU_DATA& data);

    //  the following two functions can be called to customize the noise covariance

    void setQ(RTFLOAT Q) {  m_Q = Q; reset();}
    void setR(RTFLOAT R) { if (R > 0) m_R = R; reset();}

private:
	void predict();
	void update();

    RTVector3 m_gyro;										// unbiased gyro data
    RTFLOAT m_timeDelta;                                    // time between predictions

    RTQuaternion m_stateQ;									// quaternion state vector
    RTQuaternion m_stateQError;                             // difference between stateQ and measuredQ

    RTFLOAT m_Q;                                            // process noise covariance
    RTMatrix4x4 m_Fk;                                       // the state transition matrix
    RTFLOAT m_R;                                            // the measurement noise covariance

    int m_sampleNumber;
};

#endif // _RTFUSIONRTQF_H
