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

#ifndef _RTIMUGD20M303DLHC_H
#define	_RTIMUGD20M303DLHC_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

//#define GD20M303DLHC_CACHE_MODE   // not reliable at the moment

//  I2C Slave Addresses

#define L3GD20_ADDRESS0             0x6a
#define L3GD20_ADDRESS1             0x6b
#define L3GD20_ID                   0xd4

#define LSM303DLHC_ACCEL_ADDRESS    0x19
#define LSM303DLHC_COMPASS_ADDRESS  0x1e

//  L3GD20 Register map

#define L3GD20_WHO_AM_I        0x0f
#define L3GD20_CTRL1           0x20
#define L3GD20_CTRL2           0x21
#define L3GD20_CTRL3           0x22
#define L3GD20_CTRL4           0x23
#define L3GD20_CTRL5           0x24
#define L3GD20_OUT_TEMP        0x26
#define L3GD20_STATUS          0x27
#define L3GD20_OUT_X_L         0x28
#define L3GD20_OUT_X_H         0x29
#define L3GD20_OUT_Y_L         0x2a
#define L3GD20_OUT_Y_H         0x2b
#define L3GD20_OUT_Z_L         0x2c
#define L3GD20_OUT_Z_H         0x2d
#define L3GD20_FIFO_CTRL       0x2e
#define L3GD20_FIFO_SRC        0x2f
#define L3GD20_IG_CFG          0x30
#define L3GD20_IG_SRC          0x31
#define L3GD20_IG_THS_XH       0x32
#define L3GD20_IG_THS_XL       0x33
#define L3GD20_IG_THS_YH       0x34
#define L3GD20_IG_THS_YL       0x35
#define L3GD20_IG_THS_ZH       0x36
#define L3GD20_IG_THS_ZL       0x37
#define L3GD20_IG_DURATION     0x38

//  Gyro sample rate defines

#define L3GD20_SAMPLERATE_95    0
#define L3GD20_SAMPLERATE_190   1
#define L3GD20_SAMPLERATE_380   2
#define L3GD20_SAMPLERATE_760   3

//  Gyro banwidth defines

#define L3GD20_BANDWIDTH_0     0
#define L3GD20_BANDWIDTH_1     1
#define L3GD20_BANDWIDTH_2     2
#define L3GD20_BANDWIDTH_3     3

//  Gyro FSR defines

#define L3GD20_FSR_250         0
#define L3GD20_FSR_500         1
#define L3GD20_FSR_2000        2

//  Gyro high pass filter defines

#define L3GD20_HPF_0           0
#define L3GD20_HPF_1           1
#define L3GD20_HPF_2           2
#define L3GD20_HPF_3           3
#define L3GD20_HPF_4           4
#define L3GD20_HPF_5           5
#define L3GD20_HPF_6           6
#define L3GD20_HPF_7           7
#define L3GD20_HPF_8           8
#define L3GD20_HPF_9           9

//  LSM303DLHC Accel Register Map

#define LSM303DLHC_CTRL1_A         0x20
#define LSM303DLHC_CTRL2_A         0x21
#define LSM303DLHC_CTRL3_A         0x22
#define LSM303DLHC_CTRL4_A         0x23
#define LSM303DLHC_CTRL5_A         0x24
#define LSM303DLHC_CTRL6_A         0x25
#define LSM303DLHC_REF_A           0x26
#define LSM303DLHC_STATUS_A        0x27
#define LSM303DLHC_OUT_X_L_A       0x28
#define LSM303DLHC_OUT_X_H_A       0x29
#define LSM303DLHC_OUT_Y_L_A       0x2a
#define LSM303DLHC_OUT_Y_H_A       0x2b
#define LSM303DLHC_OUT_Z_L_A       0x2c
#define LSM303DLHC_OUT_Z_H_A       0x2d
#define LSM303DLHC_FIFO_CTRL_A     0x2e
#define LSM303DLHC_FIFO_SRC_A      0x2f

//  LSM303DLHC Compass Register Map

#define LSM303DLHC_CRA_M            0x00
#define LSM303DLHC_CRB_M            0x01
#define LSM303DLHC_CRM_M            0x02
#define LSM303DLHC_OUT_X_H_M        0x03
#define LSM303DLHC_OUT_X_L_M        0x04
#define LSM303DLHC_OUT_Y_H_M        0x05
#define LSM303DLHC_OUT_Y_L_M        0x06
#define LSM303DLHC_OUT_Z_H_M        0x07
#define LSM303DLHC_OUT_Z_L_M        0x08
#define LSM303DLHC_STATUS_M         0x09
#define LSM303DLHC_TEMP_OUT_L_M     0x31
#define LSM303DLHC_TEMP_OUT_H_M     0x32

//  Accel sample rate defines

#define LSM303DLHC_ACCEL_SAMPLERATE_1       1
#define LSM303DLHC_ACCEL_SAMPLERATE_10      2
#define LSM303DLHC_ACCEL_SAMPLERATE_25      3
#define LSM303DLHC_ACCEL_SAMPLERATE_50      4
#define LSM303DLHC_ACCEL_SAMPLERATE_100     5
#define LSM303DLHC_ACCEL_SAMPLERATE_200     6
#define LSM303DLHC_ACCEL_SAMPLERATE_400     7

//  Accel FSR

#define LSM303DLHC_ACCEL_FSR_2     0
#define LSM303DLHC_ACCEL_FSR_4     1
#define LSM303DLHC_ACCEL_FSR_8     2
#define LSM303DLHC_ACCEL_FSR_16    3

//  Compass sample rate defines

#define LSM303DLHC_COMPASS_SAMPLERATE_0_75      0
#define LSM303DLHC_COMPASS_SAMPLERATE_1_5       1
#define LSM303DLHC_COMPASS_SAMPLERATE_3         2
#define LSM303DLHC_COMPASS_SAMPLERATE_7_5       3
#define LSM303DLHC_COMPASS_SAMPLERATE_15        4
#define LSM303DLHC_COMPASS_SAMPLERATE_30        5
#define LSM303DLHC_COMPASS_SAMPLERATE_75        6
#define LSM303DLHC_COMPASS_SAMPLERATE_220       7

//  Compass FSR

#define LSM303DLHC_COMPASS_FSR_1_3      1
#define LSM303DLHC_COMPASS_FSR_1_9      2
#define LSM303DLHC_COMPASS_FSR_2_5      3
#define LSM303DLHC_COMPASS_FSR_4        4
#define LSM303DLHC_COMPASS_FSR_4_7      5
#define LSM303DLHC_COMPASS_FSR_5_6      6
#define LSM303DLHC_COMPASS_FSR_8_1      7

#ifdef GD20M303DLHC_CACHE_MODE

//  Cache defs

#define GD20M303DLHC_FIFO_CHUNK_SIZE    6                       // 6 bytes of gyro data
#define GD20M303DLHC_FIFO_THRESH        16                      // threshold point in fifo
#define GD20M303DLHC_CACHE_BLOCK_COUNT  16                      // number of cache blocks

typedef struct
{
    unsigned char data[GD20M303DLHC_FIFO_THRESH * GD20M303DLHC_FIFO_CHUNK_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char accel[6];                                 // the raw accel readings for the block
    unsigned char compass[6];                               // the raw compass readings for the block

} GD20M303DLHC_CACHE_BLOCK;

#endif

class RTIMUGD20M303DLHC : public RTIMU
{
public:
    RTIMUGD20M303DLHC(RTIMUSettings *settings);
    ~RTIMUGD20M303DLHC();

    virtual const char *IMUName() { return "L3GD20 + LSM303DLHC"; }
    virtual int IMUType() { return RTIMU_TYPE_GD20M303DLHC; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroSampleRate();
    bool setGyroCTRL2();
    bool setGyroCTRL4();
    bool setGyroCTRL5();
    bool setAccelCTRL1();
    bool setAccelCTRL4();
    bool setCompassCRA();
    bool setCompassCRB();
    bool setCompassCRM();

    unsigned char m_gyroSlaveAddr;                          // I2C address of L3GD20
    unsigned char m_accelSlaveAddr;                         // I2C address of LSM303DLHC accel
    unsigned char m_compassSlaveAddr;                       // I2C address of LSM303DLHC compass
    unsigned char m_bus;                                    // I2C bus (usually 1 for Raspberry Pi for example)

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScaleXY;
    RTFLOAT m_compassScaleZ;

#ifdef GD20M303DLHC_CACHE_MODE
    bool m_firstTime;                                       // if first sample

    GD20M303DLHC_CACHE_BLOCK m_cache[GD20M303DLHC_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif
};

#endif // _RTIMUGD20M303DLHC_H
