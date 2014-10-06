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

//  GD20HM303D settings keys

#define RTIMULIB_GD20HM303D_GYRO_SAMPLERATE   "GD20HM303DGyroSampleRate"
#define RTIMULIB_GD20HM303D_GYRO_BW           "GD20HM303DGyroBW"
#define RTIMULIB_GD20HM303D_GYRO_HPF          "GD20HM303DGyroHpf"
#define RTIMULIB_GD20HM303D_GYRO_FSR          "GD20HM303DGyroFsr"

#define RTIMULIB_GD20HM303D_ACCEL_SAMPLERATE  "GD20HM303DAccelSampleRate"
#define RTIMULIB_GD20HM303D_ACCEL_FSR         "GD20HM303DAccelFsr"
#define RTIMULIB_GD20HM303D_ACCEL_LPF         "GD20HM303DAccelLpf"

#define RTIMULIB_GD20HM303D_COMPASS_SAMPLERATE "GD20HM303DCompassSampleRate"
#define RTIMULIB_GD20HM303D_COMPASS_FSR       "GD20HM303DCompassFsr"


//  GD20M303DLHC settings keys

#define RTIMULIB_GD20M303DLHC_GYRO_SAMPLERATE   "GD20M303DLHCGyroSampleRate"
#define RTIMULIB_GD20M303DLHC_GYRO_BW           "GD20M303DLHCGyroBW"
#define RTIMULIB_GD20M303DLHC_GYRO_HPF          "GD20M303DLHCGyroHpf"
#define RTIMULIB_GD20M303DLHC_GYRO_FSR          "GD20M303DLHCGyroFsr"

#define RTIMULIB_GD20M303DLHC_ACCEL_SAMPLERATE  "GD20M303DLHCAccelSampleRate"
#define RTIMULIB_GD20M303DLHC_ACCEL_FSR         "GD20M303DLHCAccelFsr"

#define RTIMULIB_GD20M303DLHC_COMPASS_SAMPLERATE "GD20M303DLHCCompassSampleRate"
#define RTIMULIB_GD20M303DLHC_COMPASS_FSR       "GD20M303DLHCCompassFsr"


//  LSM9DS0 settings keys

#define RTIMULIB_LSM9DS0_GYRO_SAMPLERATE   "LSM9DS0GyroSampleRate"
#define RTIMULIB_LSM9DS0_GYRO_BW           "LSM9DS0GyroBW"
#define RTIMULIB_LSM9DS0_GYRO_HPF          "LSM9DS0GyroHpf"
#define RTIMULIB_LSM9DS0_GYRO_FSR          "LSM9DS0GyroFsr"

#define RTIMULIB_LSM9DS0_ACCEL_SAMPLERATE  "LSM9DS0AccelSampleRate"
#define RTIMULIB_LSM9DS0_ACCEL_FSR         "LSM9DS0AccelFsr"
#define RTIMULIB_LSM9DS0_ACCEL_LPF         "LSM9DS0AccelLpf"

#define RTIMULIB_LSM9DS0_COMPASS_SAMPLERATE "LSM9DS0CompassSampleRate"
#define RTIMULIB_LSM9DS0_COMPASS_FSR       "LSM9DS0CompassFsr"

//  Gyro bias keys

#define RTIMULIB_GYRO_BIAS_VALID            "GyroBiasValid"
#define RTIMULIB_GYRO_BIAS_X                "GyroBiasX"
#define RTIMULIB_GYRO_BIAS_Y                "GyroBiasY"
#define RTIMULIB_GYRO_BIAS_Z                "GyroBiasZ"

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

    bool m_gyroBiasValid;                                   // true if the recorded gyro bias is valid
    RTVector3 m_gyroBias;                                   // the recorded gyro bias

    //  IMU-specific vars

    //  MPU9150

    int m_MPU9150GyroAccelSampleRate;                       // the sample rate (samples per second) for gyro and accel
    int m_MPU9150CompassSampleRate;                         // same for the compass
    int m_MPU9150GyroAccelLpf;                              // low pass filter code for the gyro and accel
    int m_MPU9150GyroFsr;                                   // FSR code for the gyro
    int m_MPU9150AccelFsr;                                  // FSR code for the accel

    //  GD20HM303D

    int m_GD20HM303DGyroSampleRate;                         // the gyro sample rate
    int m_GD20HM303DGyroBW;                                 // the gyro bandwidth code
    int m_GD20HM303DGyroHpf;                                // the gyro high pass filter cutoff code
    int m_GD20HM303DGyroFsr;                                // the gyro full scale range

    int m_GD20HM303DAccelSampleRate;                        // the accel sample rate
    int m_GD20HM303DAccelFsr;                               // the accel full scale range
    int m_GD20HM303DAccelLpf;                               // the accel low pass filter

    int m_GD20HM303DCompassSampleRate;                      // the compass sample rate
    int m_GD20HM303DCompassFsr;                             // the compass full scale range

    //  GD20M303DLHC

    int m_GD20M303DLHCGyroSampleRate;                       // the gyro sample rate
    int m_GD20M303DLHCGyroBW;                               // the gyro bandwidth code
    int m_GD20M303DLHCGyroHpf;                              // the gyro high pass filter cutoff code
    int m_GD20M303DLHCGyroFsr;                              // the gyro full scale range

    int m_GD20M303DLHCAccelSampleRate;                      // the accel sample rate
    int m_GD20M303DLHCAccelFsr;                             // the accel full scale range

    int m_GD20M303DLHCCompassSampleRate;                    // the compass sample rate
    int m_GD20M303DLHCCompassFsr;                           // the compass full scale range

    //  LSM9DS0

    int m_LSM9DS0GyroSampleRate;                            // the gyro sample rate
    int m_LSM9DS0GyroBW;                                    // the gyro bandwidth code
    int m_LSM9DS0GyroHpf;                                   // the gyro high pass filter cutoff code
    int m_LSM9DS0GyroFsr;                                   // the gyro full scale range

    int m_LSM9DS0AccelSampleRate;                           // the accel sample rate
    int m_LSM9DS0AccelFsr;                                  // the accel full scale range
    int m_LSM9DS0AccelLpf;                                  // the accel low pass filter

    int m_LSM9DS0CompassSampleRate;                         // the compass sample rate
    int m_LSM9DS0CompassFsr;                                // the compass full scale range

private:
    void setBlank();
    void setComment(const char *comment);
    void setValue(const char *key, const bool val);
    void setValue(const char *key, const int val);
    void setValue(const char *key, const RTFLOAT val);

    char m_filename[256];                                    // the settings file name

    FILE *m_fd;
};

#endif // _RTIMUSETTINGS_H

