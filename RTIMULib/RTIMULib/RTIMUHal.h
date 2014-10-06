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

#ifndef _RTIMUHAL_H
#define	_RTIMUHAL_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>

#ifndef HAL_QUITE
#define HAL_INFO(m) { printf("%s", m); fflush(stdout); }
#define HAL_INFO1(m, x) { printf(m, x); fflush(stdout); }
#define HAL_INFO2(m, x, y) { printf(m, x, y); fflush(stdout); }
#define HAL_INFO3(m, x, y, z) { printf(m, x, y, z); fflush(stdout); }
#define HAL_INFO4(m, x, y, z, a) { printf(m, x, y, z, a); fflush(stdout); }
#define HAL_INFO5(m, x, y, z, a, b) { printf(m, x, y, z, a, b); fflush(stdout); }
#define HAL_ERROR(m)    fprintf(stderr, m);
#define HAL_ERROR1(m, x)    fprintf(stderr, m, x);
#define HAL_ERROR2(m, x, y)    fprintf(stderr, m, x, y);
#define HAL_ERROR3(m, x, y, z)    fprintf(stderr, m, x, y, z);

#else

#define HAL_INFO(m) 
#define HAL_INFO1(m, x)
#define HAL_INFO2(m, x, y)
#define HAL_INFO3(m, x, y, z)
#define HAL_INFO4(m, x, y, z, a)
#define HAL_INFO5(m, x, y, z, a, b)
#define HAL_ERROR(m)
#define HAL_ERROR1(m, x)
#define HAL_ERROR2(m, x, y)
#define HAL_ERROR3(m, x, y, z)

#endif

#define MAX_WRITE_LEN 511

class RTIMUHal
{
public:
    RTIMUHal();
    virtual ~RTIMUHal();

protected:
    void setI2CBus(unsigned char port);
    bool I2COpen();
    void I2CClose();
    bool I2CRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                 unsigned char *data, const char *errorMsg);
    bool I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg);
    bool I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char length, unsigned char const *data, const char *errorMsg);
    bool I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char const data, const char *errorMsg);

    void delayMs(int milliSeconds);

private:
    int m_I2C;

    unsigned char m_I2CBus;
    unsigned char m_currentSlave;
};

#endif // _RTIMUHAL_H
