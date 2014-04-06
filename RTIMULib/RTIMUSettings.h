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

#ifndef _RTIMUSETTINGS_H
#define _RTIMUSETTINGS_H

#include "RTMath.h"
#include "RTIMUHal.h"

//  IMU type codes

#define RTIMU_TYPE_AUTODISCOVER             0                   // audodiscover the IMU
#define RTIMU_TYPE_NULL                     1                   // if no physical hardware
#define RTIMU_TYPE_MPU9150                  2                   // InvenSense MPU9150
#define RTIMU_TYPE_GD20M303                 3                   // STM L3GD20H/LSM303D

//  these defines describe the various fusion filter options

#define RTFUSION_TYPE_NULL                  0                   // just a dummy to keep things happy if not needed
#define RTFUSION_TYPE_KALMANSTATE4          1                   // kalman state is the quaternion pose
#define RTFUSION_TYPE_KALMANSTATE7          2                   // same as above but also tracks gyro bias

//  Settings keys

#define RTIMULIB_IMU_TYPE                   "IMUType"
#define RTIMULIB_FUSION_TYPE                "FusionType"
#define RTIMULIB_I2C_SLAVEADDRESS           "I2CSlaveAddress"
#define RTIMULIB_I2C_BUS                    "I2CBus"

//  MPU9150 settings keys

#define RTIMULIB_MPU9150_GYROACCEL_SAMPLERATE "MPU9150GyroAccelSampleRate"
#define RTIMULIB_MPU9150_COMPASS_SAMPLERATE "MPU9150CompassSampleRate"
#define RTIMULIB_MPU9150_GYROACCEL_LPF      "MPU9150GyroAccelLpf"
#define RTIMULIB_MPU9150_GYRO_FSR           "MPU9150GyroFSR"
#define RTIMULIB_MPU9150_ACCEL_FSR          "MPU9150AccelFSR"

//  GD20M303 settings keys

#define RTIMULIB_GD20M303_GYRO_SAMPLERATE   "GD20M303GyroSampleRate"
#define RTIMULIB_GD20M303_ACCEL_SAMPLERATE  "GD20M303AccelSampleRate"
#define RTIMULIB_GD20M303_COMPASS_SAMPLERATE "GD20M303CompassSampleRate"
#define RTIMULIB_GD20M303_GYRO_BW           "GD20M303GyroBW"
#define RTIMULIB_GD20M303_GYRO_HPF          "GD20M303GyroHpf"
#define RTIMULIB_GD20M303_GYRO_FSR          "GD20M303GyroFsr"
#define RTIMULIB_GD20M303_ACCEL_LPF         "GD20M303AccelLpf"


//  Compass calibration settings keys

#define RTIMULIB_COMPASSCAL_VALID           "CompassCalValid"
#define RTIMULIB_COMPASSCAL_MINX            "CompassCalMinX"
#define RTIMULIB_COMPASSCAL_MAXX            "CompassCalMaxX"
#define RTIMULIB_COMPASSCAL_MINY            "CompassCalMinY"
#define RTIMULIB_COMPASSCAL_MAXY            "CompassCalMaxY"
#define RTIMULIB_COMPASSCAL_MINZ            "CompassCalMinZ"
#define RTIMULIB_COMPASSCAL_MAXZ            "CompassCalMaxZ"

class RTIMUSettings : public RTIMUHal
{
public:
    RTIMUSettings(const char *productType = "RTIMULib");

    //  This function tries to find an IMU. It stops at the first valid one
    //  and return true or else false

    bool discoverIMU(int& imuType, unsigned char& slaveAddress);

    //  This function loads the local variables from the settings file or uses defaults

    bool loadSettings();

    //  This function saves the local variables to the settings file

    bool saveSettings();

    //  These are the local variables

    int m_imuType;                                          // type code of imu in use
    int m_fusionType;                                       // fusion algorithm type code

    unsigned char m_I2CSlaveAddress;                        // I2C slave address of the imu
    unsigned char m_I2CBus;                                 // I2C bus of the imu (eg 1 for Raspberry Pi usually)

    bool m_compassCalValid;                                 // true if there is valid compass calibration data
    RTVector3 m_compassCalMin;                              // the minimum values
    RTVector3 m_compassCalMax;                              // the maximum values

    //  IMU-specific vars

    //  MPU9150

    int m_MPU9150GyroAccelSampleRate;                       // the sample rate (samples per second) for gyro and accel
    int m_MPU9150CompassSampleRate;                         // same for the compass
    int m_MPU9150GyroAccelLpf;                              // low pass filter code for the gyro and accel
    int m_MPU9150GyroFsr;                                   // FSR code for the gyro
    int m_MPU9150AccelFsr;                                  // FSR code for the accel

    //  GD20M303

    int m_GD20M303GyroSampleRate;                           // the gyro sample rate
    int m_GD20M303AccelSampleRate;                          // the accel sample rate
    int m_GD20M303CompassSampleRate;                        // the compass sample rate
    int m_GD20M303GyroBW;                                   // the gyro bandwidth code
    int m_GD20M303GyroHpf;                                  // the gyro high pass filter cutoff code
    int m_GD20M303GyroFsr;                                  // the gyro full scale range

private:
    void setValue(const char *key, const bool val);
    void setValue(const char *key, const int val);
    void setValue(const char *key, const RTFLOAT val);

    char m_filename[256];                                    // the settings file name

    FILE *m_fd;
};

#endif // _RTIMUSETTINGS_H

