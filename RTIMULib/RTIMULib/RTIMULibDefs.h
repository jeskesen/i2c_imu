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

#ifndef _RTIMULIBDEFS_H
#define	_RTIMULIBDEFS_H

#include "RTMath.h"

//  IMU type codes

#define RTIMU_TYPE_AUTODISCOVER             0                   // audodiscover the IMU
#define RTIMU_TYPE_NULL                     1                   // if no physical hardware
#define RTIMU_TYPE_MPU9150                  2                   // InvenSense MPU9150
#define RTIMU_TYPE_GD20HM303D               3                   // STM L3GD20H/LSM303D (Pololu Altimu)
#define RTIMU_TYPE_GD20M303DLHC             4                   // STM L3GD20/LSM303DHLC (Adafruit IMU)
#define RTIMU_TYPE_LSM9DS0                  5                   // STM LSM9DS0 (eg Sparkfun IMU)

//  these defines describe the various fusion filter options

#define RTFUSION_TYPE_NULL                  0                   // just a dummy to keep things happy if not needed
#define RTFUSION_TYPE_KALMANSTATE4          1                   // kalman state is the quaternion pose
#define RTFUSION_TYPE_RTQF                  2                   // RT quaternion fusion

#define RTFUSION_TYPE_COUNT                 3                   // number of fusion algorithm types

//  This is a convenience structure that can be used to pass IMU data around

typedef struct
{
    uint64_t timestamp;
    bool fusionPoseValid;
    RTVector3 fusionPose;
    bool fusionQPoseValid;
    RTQuaternion fusionQPose;
    bool gyroValid;
    RTVector3 gyro;
    bool accelValid;
    RTVector3 accel;
    bool compassValid;
    RTVector3 compass;
    bool pressureValid;
    RTFLOAT pressure;
    bool temperatureValid;
    RTFLOAT temperature;
    bool humidityValid;
    RTFLOAT humidity;
} RTIMU_DATA;

#endif // _RTIMULIBDEFS_H
