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

#include "RTIMU.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>


RTIMUHal::RTIMUHal()
{
    m_I2CBus = 255;
    m_currentSlave = 255;
    m_I2C = -1;
}

RTIMUHal::~RTIMUHal()
{
    I2CClose();
}

void RTIMUHal::setI2CBus(unsigned char bus)
{
    m_I2CBus = bus;
}

bool RTIMUHal::I2COpen()
{
    char buf[32];

    if (m_I2C >= 0)
        return true;

    if (m_I2CBus == 255) {
        HAL_ERROR("No I2C bus has been set\n");
        return false;
    }
    sprintf(buf, "/dev/i2c-%d", m_I2CBus);
    m_I2C = open(buf, O_RDWR);
    if (m_I2C < 0) {
        HAL_ERROR1("Failed to open I2C bus %d\n", m_I2CBus);
        return false;
    }
    return true;
}

void RTIMUHal::I2CClose()
{
    if (m_I2C >= 0) {
        close(m_I2C);
        m_I2C = -1;
        m_currentSlave = 255;
    }
}

bool RTIMUHal::I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char const data, const char *errorMsg)
{
    return I2CWrite(slaveAddr, regAddr, 1, &data, errorMsg);
}

bool RTIMUHal::I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char length, unsigned char const *data, const char *errorMsg)
{
    int result, i;
    unsigned char txBuff[MAX_WRITE_LEN + 1];

    if (!I2CSelectSlave(slaveAddr, errorMsg))
        return false;

    if (length == 0) {
        result = write(m_I2C, &regAddr, 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR1(" I2C write of regAddr failed - %s\n", errorMsg);
            return false;
        } else if (result != 1) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR1(" I2C write of regAddr failed (nothing written) - %s\n", errorMsg);
            return false;
        }
    } else {
        txBuff[0] = regAddr;

        for (i = 0; i < length; i++)
            txBuff[i+1] = data[i];

        result = write(m_I2C, txBuff, length + 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("I2C data write of %d bytes failed - %s\n", length, errorMsg);
            return false;
        } else if (result < (int)length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("I2C data write of %d bytes failed, ony %d written - %s\n", length, result, errorMsg);
            return false;
        }
    }

    return true;
}


bool RTIMUHal::I2CRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    int tries, result, total;

    if (!I2CWrite(slaveAddr, regAddr, 0, NULL, errorMsg))
        return false;

    total = 0;
    tries = 0;

    while ((total < length) && (tries < 5)) {
        result = read(m_I2C, data + total, length - total);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("I2C read error from %d, %d - %s\n", slaveAddr, regAddr, errorMsg);
            return false;
        }

        total += result;

        if (total == length)
            break;

        delayMs(10);
        tries++;
    }

    if (total < length) {
        if (strlen(errorMsg) > 0)
            HAL_ERROR3("I2C read from %d, %d failed - %s\n", slaveAddr, regAddr, errorMsg);
        return false;
    }
    return true;
}


bool RTIMUHal::I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg)
{
    if (m_currentSlave == slaveAddr)
        return true;

    if (!I2COpen()) {
        HAL_ERROR1("Failed to open I2C port - %s\n", errorMsg);
        return false;
    }

    if (ioctl(m_I2C, I2C_SLAVE, slaveAddr) < 0) {
        HAL_ERROR2("I2C slave select %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }

    m_currentSlave = slaveAddr;

    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
    usleep(1000 * milliSeconds);
}


