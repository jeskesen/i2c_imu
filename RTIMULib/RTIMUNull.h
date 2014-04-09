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

#ifndef _RTIMUNULL_H
#define	_RTIMUNULL_H

//  IMUNull is a dummy IMU that assumes sensor data is coming from elsewhere,
//  for example, across a network.
//
//  Call IMUInit in the normal way. Then for every update, call setIMUData and then IMURead
//  to kick the kalman filter.

#include "RTIMU.h"

class RTIMUSettings;

class RTIMUNull : public RTIMU
{
public:
    RTIMUNull(RTIMUSettings *settings);
    ~RTIMUNull();

    // The timestamp parameter is assumed to be from RTMath::currentUSecsSinceEpoch()

    void setIMUData(const RTIMU_DATA& data);

    virtual const char *IMUName() { return "Null IMU"; }
    virtual int IMUType() { return RTIMU_TYPE_NULL; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();
    virtual bool IMUGyroBiasValid() { return true; }

private:
    uint64_t m_timestamp;
};

#endif // _RTIMUNULL_H
