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

#ifndef _RTIMUMPU9150_H
#define	_RTIMUMPU9150_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

#define MPU9150_CACHE_MODE

//  MPU9150 I2C Slave Addresses

#define MPU9150_ADDRESS0            0x68
#define MPU9150_ADDRESS1            0x69
#define MPU9150_ID                  0x68

#define AK8975_ADDRESS              0x0c

//  Register map

#define MPU9150_YG_OFFS_TC          0x01
#define MPU9150_SMPRT_DIV           0x19
#define MPU9150_LPF_CONFIG          0x1a
#define MPU9150_GYRO_CONFIG         0x1b
#define MPU9150_ACCEL_CONFIG        0x1c
#define MPU9150_FIFO_EN             0x23
#define MPU9150_I2C_MST_CTRL        0x24
#define MPU9150_I2C_SLV0_ADDR       0x25
#define MPU9150_I2C_SLV0_REG        0x26
#define MPU9150_I2C_SLV0_CTRL       0x27
#define MPU9150_I2C_SLV1_ADDR       0x28
#define MPU9150_I2C_SLV1_REG        0x29
#define MPU9150_I2C_SLV1_CTRL       0x2a
#define MPU9150_I2C_SLV4_CTRL       0x34
#define MPU9150_INT_PIN_CFG         0x37
#define MPU9150_INT_ENABLE          0x38
#define MPU9150_INT_STATUS          0x3a
#define MPU9150_ACCEL_XOUT_H        0x3b
#define MPU9150_GYRO_XOUT_H         0x43
#define MPU9150_EXT_SENS_DATA_00    0x49
#define MPU9150_I2C_SLV1_DO         0x64
#define MPU9150_I2C_MST_DELAY_CTRL  0x67
#define MPU9150_USER_CTRL           0x6a
#define MPU9150_PWR_MGMT_1          0x6b
#define MPU9150_PWR_MGMT_2          0x6c
#define MPU9150_FIFO_COUNT_H        0x72
#define MPU9150_FIFO_R_W            0x74
#define MPU9150_WHO_AM_I            0x75

//  sample rate defines (applies to gyros and accels, not mags)

#define MPU9150_SAMPLERATE_MIN      5                      // 5 samples per second is the lowest
#define MPU9150_SAMPLERATE_MAX      1000                   // 1000 samples per second is the absolute maximum

//  compass rate defines

#define MPU9150_COMPASSRATE_MIN     1                      // 1 samples per second is the lowest
#define MPU9150_COMPASSRATE_MAX     100                    // 100 samples per second is maximum

//  LPF options (gyros and accels)

#define MPU9150_LPF_256             0                       // gyro: 256Hz, accel: 260Hz
#define MPU9150_LPF_188             1                       // gyro: 188Hz, accel: 184Hz
#define MPU9150_LPF_98              2                       // gyro: 98Hz, accel: 98Hz
#define MPU9150_LPF_42              3                       // gyro: 42Hz, accel: 44Hz
#define MPU9150_LPF_20              4                       // gyro: 20Hz, accel: 21Hz
#define MPU9150_LPF_10              5                       // gyro: 10Hz, accel: 10Hz
#define MPU9150_LPF_5               6                       // gyro: 5Hz, accel: 5Hz

//  Gyro FSR options

#define MPU9150_GYROFSR_250         0                       // +/- 250 degrees per second
#define MPU9150_GYROFSR_500         8                       // +/- 500 degrees per second
#define MPU9150_GYROFSR_1000        0x10                    // +/- 1000 degrees per second
#define MPU9150_GYROFSR_2000        0x18                    // +/- 2000 degrees per second

//  Accel FSR options

#define MPU9150_ACCELFSR_2          0                       // +/- 2g
#define MPU9150_ACCELFSR_4          8                       // +/- 4g
#define MPU9150_ACCELFSR_8          0x10                    // +/- 8g
#define MPU9150_ACCELFSR_16         0x18                    // +/- 16g


//  AK8975 compass registers

#define AK8975_DEVICEID             0x0                     // the device ID
#define AK8975_ST1                  0x02                    // status 1
#define AK8975_CNTL                 0x0a                    // control reg
#define AK8975_ASAX                 0x10                    // start of the fuse ROM data

//  FIFO transfer size

#define MPU9150_FIFO_CHUNK_SIZE     12                      // gyro and accels take 12 bytes

#ifdef MPU9150_CACHE_MODE

//  Cache mode defines

#define MPU9150_CACHE_SIZE          16                      // number of chunks in a block
#define MPU9150_CACHE_BLOCK_COUNT   16                      // number of cache blocks

typedef struct
{
    unsigned char data[MPU9150_FIFO_CHUNK_SIZE * MPU9150_CACHE_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char compass[8];                               // the raw compass readings for the block

} MPU9150_CACHE_BLOCK;

#endif


class RTIMUMPU9150 : public RTIMU
{
public:
    RTIMUMPU9150(RTIMUSettings *settings);
    ~RTIMUMPU9150();

    bool setLpf(unsigned char lpf);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelFsr(unsigned char fsr);

    virtual const char *IMUName() { return "MPU-9150"; }
    virtual int IMUType() { return RTIMU_TYPE_MPU9150; }
    virtual bool IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

private:
    bool bypassOn();                                        // talk to compass
    bool bypassOff();                                       // talk to MPU9150
    bool setSampleRate();
    bool setCompassRate();
    bool resetFifo();

    bool m_firstTime;                                       // if first sample

    unsigned char m_slaveAddr;                              // I2C address of MPU9150
    unsigned char m_bus;                                    // I2C bus (usually 1 for Raspberry Pi for example)

    unsigned char m_lpf;                                    // low pass filter setting
    int m_compassRate;                                      // compass sample rate in Hz
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;

    RTFLOAT m_compassAdjust[3];                             // the compass fuse ROM values converted for use

#ifdef MPU9150_CACHE_MODE

    MPU9150_CACHE_BLOCK m_cache[MPU9150_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif

};

#endif // _RTIMUMPU9150_H
