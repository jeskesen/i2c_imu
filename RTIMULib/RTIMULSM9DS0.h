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

#ifndef _RTIMULSM9DS0_H
#define	_RTIMULSM9DS0_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

//#define LSM9DS0_CACHE_MODE   // not reliable at the moment

//  I2C Slave Addresses

#define LSM9DS0_GYRO_ADDRESS0       0x6a
#define LSM9DS0_GYRO_ADDRESS1       0x6b
#define LSM9DS0_GYRO_ID             0xd4

#define LSM9DS0_ACCELMAG_ADDRESS0   0x1c
#define LSM9DS0_ACCELMAG_ADDRESS1   0x1d
#define LSM9DS0_ACCELMAG_ID         0x49

//  L3GD20 Register map

#define LSM9DS0_GYRO_WHO_AM_I       0x0f
#define LSM9DS0_GYRO_CTRL1          0x20
#define LSM9DS0_GYRO_CTRL2          0x21
#define LSM9DS0_GYRO_CTRL3          0x22
#define LSM9DS0_GYRO_CTRL4          0x23
#define LSM9DS0_GYRO_CTRL5          0x24
#define LSM9DS0_GYRO_OUT_TEMP       0x26
#define LSM9DS0_GYRO_STATUS         0x27
#define LSM9DS0_GYRO_OUT_X_L        0x28
#define LSM9DS0_GYRO_OUT_X_H        0x29
#define LSM9DS0_GYRO_OUT_Y_L        0x2a
#define LSM9DS0_GYRO_OUT_Y_H        0x2b
#define LSM9DS0_GYRO_OUT_Z_L        0x2c
#define LSM9DS0_GYRO_OUT_Z_H        0x2d
#define LSM9DS0_GYRO_FIFO_CTRL      0x2e
#define LSM9DS0_GYRO_FIFO_SRC       0x2f
#define LSM9DS0_GYRO_IG_CFG         0x30
#define LSM9DS0_GYRO_IG_SRC         0x31
#define LSM9DS0_GYRO_IG_THS_XH      0x32
#define LSM9DS0_GYRO_IG_THS_XL      0x33
#define LSM9DS0_GYRO_IG_THS_YH      0x34
#define LSM9DS0_GYRO_IG_THS_YL      0x35
#define LSM9DS0_GYRO_IG_THS_ZH      0x36
#define LSM9DS0_GYRO_IG_THS_ZL      0x37
#define LSM9DS0_GYRO_IG_DURATION    0x38

//  Gyro sample rate defines

#define LSM9DS0_GYRO_SAMPLERATE_95  0
#define LSM9DS0_GYRO_SAMPLERATE_190 1
#define LSM9DS0_GYRO_SAMPLERATE_380 2
#define LSM9DS0_GYRO_SAMPLERATE_760 3

//  Gyro banwidth defines

#define LSM9DS0_GYRO_BANDWIDTH_0    0
#define LSM9DS0_GYRO_BANDWIDTH_1    1
#define LSM9DS0_GYRO_BANDWIDTH_2    2
#define LSM9DS0_GYRO_BANDWIDTH_3    3

//  Gyro FSR defines

#define LSM9DS0_GYRO_FSR_250        0
#define LSM9DS0_GYRO_FSR_500        1
#define LSM9DS0_GYRO_FSR_2000       2

//  Gyro high pass filter defines

#define LSM9DS0_GYRO_HPF_0          0
#define LSM9DS0_GYRO_HPF_1          1
#define LSM9DS0_GYRO_HPF_2          2
#define LSM9DS0_GYRO_HPF_3          3
#define LSM9DS0_GYRO_HPF_4          4
#define LSM9DS0_GYRO_HPF_5          5
#define LSM9DS0_GYRO_HPF_6          6
#define LSM9DS0_GYRO_HPF_7          7
#define LSM9DS0_GYRO_HPF_8          8
#define LSM9DS0_GYRO_HPF_9          9

//  Accel/Mag Register Map

#define LSM9DS0_TEMP_OUT_L      0x05
#define LSM9DS0_TEMP_OUT_H      0x06
#define LSM9DS0_STATUS_M        0x07
#define LSM9DS0_OUT_X_L_M       0x08
#define LSM9DS0_OUT_X_H_M       0x09
#define LSM9DS0_OUT_Y_L_M       0x0a
#define LSM9DS0_OUT_Y_H_M       0x0b
#define LSM9DS0_OUT_Z_L_M       0x0c
#define LSM9DS0_OUT_Z_H_M       0x0d
#define LSM9DS0_WHO_AM_I        0x0f
#define LSM9DS0_INT_CTRL_M      0x12
#define LSM9DS0_INT_SRC_M       0x13
#define LSM9DS0_INT_THS_L_M     0x14
#define LSM9DS0_INT_THS_H_M     0x15
#define LSM9DS0_OFFSET_X_L_M    0x16
#define LSM9DS0_OFFSET_X_H_M    0x17
#define LSM9DS0_OFFSET_Y_L_M    0x18
#define LSM9DS0_OFFSET_Y_H_M    0x19
#define LSM9DS0_OFFSET_Z_L_M    0x1a
#define LSM9DS0_OFFSET_Z_H_M    0x1b
#define LSM9DS0_REFERENCE_X     0x1c
#define LSM9DS0_REFERENCE_Y     0x1d
#define LSM9DS0_REFERENCE_Z     0x1e
#define LSM9DS0_CTRL0           0x1f
#define LSM9DS0_CTRL1           0x20
#define LSM9DS0_CTRL2           0x21
#define LSM9DS0_CTRL3           0x22
#define LSM9DS0_CTRL4           0x23
#define LSM9DS0_CTRL5           0x24
#define LSM9DS0_CTRL6           0x25
#define LSM9DS0_CTRL7           0x26
#define LSM9DS0_STATUS_A        0x27
#define LSM9DS0_OUT_X_L_A       0x28
#define LSM9DS0_OUT_X_H_A       0x29
#define LSM9DS0_OUT_Y_L_A       0x2a
#define LSM9DS0_OUT_Y_H_A       0x2b
#define LSM9DS0_OUT_Z_L_A       0x2c
#define LSM9DS0_OUT_Z_H_A       0x2d
#define LSM9DS0_FIFO_CTRL       0x2e
#define LSM9DS0_FIFO_SRC        0x2f
#define LSM9DS0_IG_CFG1         0x30
#define LSM9DS0_IG_SRC1         0x31
#define LSM9DS0_IG_THS1         0x32
#define LSM9DS0_IG_DUR1         0x33
#define LSM9DS0_IG_CFG2         0x34
#define LSM9DS0_IG_SRC2         0x35
#define LSM9DS0_IG_THS2         0x36
#define LSM9DS0_IG_DUR2         0x37
#define LSM9DS0_CLICK_CFG       0x38
#define LSM9DS0_CLICK_SRC       0x39
#define LSM9DS0_CLICK_THS       0x3a
#define LSM9DS0_TIME_LIMIT      0x3b
#define LSM9DS0_TIME_LATENCY    0x3c
#define LSM9DS0_TIME_WINDOW     0x3d
#define LSM9DS0_ACT_THIS        0x3e
#define LSM9DS0_ACT_DUR         0x3f

//  Accel sample rate defines

#define LSM9DS0_ACCEL_SAMPLERATE_3_125 1
#define LSM9DS0_ACCEL_SAMPLERATE_6_25 2
#define LSM9DS0_ACCEL_SAMPLERATE_12_5 3
#define LSM9DS0_ACCEL_SAMPLERATE_25   4
#define LSM9DS0_ACCEL_SAMPLERATE_50   5
#define LSM9DS0_ACCEL_SAMPLERATE_100  6
#define LSM9DS0_ACCEL_SAMPLERATE_200  7
#define LSM9DS0_ACCEL_SAMPLERATE_400  8
#define LSM9DS0_ACCEL_SAMPLERATE_800  9
#define LSM9DS0_ACCEL_SAMPLERATE_1600 10

//  Accel FSR

#define LSM9DS0_ACCEL_FSR_2     0
#define LSM9DS0_ACCEL_FSR_4     1
#define LSM9DS0_ACCEL_FSR_6     2
#define LSM9DS0_ACCEL_FSR_8     3
#define LSM9DS0_ACCEL_FSR_16    4

//  Accel filter bandwidth

#define LSM9DS0_ACCEL_LPF_773   0
#define LSM9DS0_ACCEL_LPF_194   1
#define LSM9DS0_ACCEL_LPF_362   2
#define LSM9DS0_ACCEL_LPF_50    3

//  Compass sample rate defines

#define LSM9DS0_COMPASS_SAMPLERATE_3_125    0
#define LSM9DS0_COMPASS_SAMPLERATE_6_25     1
#define LSM9DS0_COMPASS_SAMPLERATE_12_5     2
#define LSM9DS0_COMPASS_SAMPLERATE_25       3
#define LSM9DS0_COMPASS_SAMPLERATE_50       4
#define LSM9DS0_COMPASS_SAMPLERATE_100      5

//  Compass FSR

#define LSM9DS0_COMPASS_FSR_2   0
#define LSM9DS0_COMPASS_FSR_4   1
#define LSM9DS0_COMPASS_FSR_8   2
#define LSM9DS0_COMPASS_FSR_12  3


#ifdef LSM9DS0_CACHE_MODE

//  Cache defs

#define LSM9DS0_FIFO_CHUNK_SIZE    6                       // 6 bytes of gyro data
#define LSM9DS0_FIFO_THRESH        16                      // threshold point in fifo
#define LSM9DS0_CACHE_BLOCK_COUNT  16                      // number of cache blocks

typedef struct
{
    unsigned char data[LSM9DS0_FIFO_THRESH * LSM9DS0_FIFO_CHUNK_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char accel[6];                                 // the raw accel readings for the block
    unsigned char compass[6];                               // the raw compass readings for the block

} LSM9DS0_CACHE_BLOCK;

#endif

class RTIMULSM9DS0 : public RTIMU
{
public:
    RTIMULSM9DS0(RTIMUSettings *settings);
    ~RTIMULSM9DS0();

    virtual const char *IMUName() { return "LSM9DS0"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM9DS0; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroSampleRate();
    bool setGyroCTRL2();
    bool setGyroCTRL4();
    bool setGyroCTRL5();
    bool setAccelCTRL1();
    bool setAccelCTRL2();
    bool setCompassCTRL5();
    bool setCompassCTRL6();
    bool setCompassCTRL7();

    unsigned char m_gyroSlaveAddr;                          // I2C address of gyro
    unsigned char m_accelCompassSlaveAddr;                  // I2C address of accel and mag
    unsigned char m_bus;                                    // I2C bus (usually 1 for Raspberry Pi for example)

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;

#ifdef LSM9DS0_CACHE_MODE
    bool m_firstTime;                                       // if first sample

    LSM9DS0_CACHE_BLOCK m_cache[LSM9DS0_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif
};

#endif // _RTIMULSM9DS0_H
