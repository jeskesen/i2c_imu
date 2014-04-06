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

#include "RTIMUSettings.h"
#include "RTIMUMPU9150.h"
#include "RTIMUGD20M303.h"

#define RATE_TIMER_INTERVAL 2

RTIMUSettings::RTIMUSettings(const char *productType)
{
    if ((strlen(productType) > 200) || (strlen(productType) == 0)) {
        HAL_ERROR("Product name too long or null - using default\n");
        strcpy(m_filename, "RTIMULib.ini");
    } else {
        sprintf(m_filename, "%s.ini", productType);
    }
    loadSettings();
}

bool RTIMUSettings::discoverIMU(int& imuType, unsigned char& slaveAddress)
{
    unsigned char result;

    setI2CBus(m_I2CBus);
    if (!I2COpen()) {
        HAL_ERROR1("Failed to open I2C bus %d\n", m_I2CBus);
        return false;
    }

    if (I2CRead(MPU9150_ADDRESS0, MPU9150_WHO_AM_I, 1, &result, "")) {
        if (result == 0x68) {
            imuType = RTIMU_TYPE_MPU9150;
            slaveAddress = MPU9150_ADDRESS0;
            I2CClose();
            HAL_INFO("Detected MPU9150 at standard address\n");
            return true;
        }
    }

    if (I2CRead(MPU9150_ADDRESS1, MPU9150_WHO_AM_I, 1, &result, "")) {
        if (result == 0x68) {
            imuType = RTIMU_TYPE_MPU9150;
            slaveAddress = MPU9150_ADDRESS1;
            I2CClose();
            HAL_INFO("Detected MPU9150 at option address\n");
            return true;
        }
    }

    if (I2CRead(L3GD20H_ADDRESS0, L3GD20H_WHO_AM_I, 1, &result, "")) {
        if (result == 0xd7) {
            imuType = RTIMU_TYPE_GD20M303;
            slaveAddress = L3GD20H_ADDRESS0;
            I2CClose();
            HAL_INFO("Detected L3GD20H at standard address\n");
            return true;
        }
    }

    if (I2CRead(L3GD20H_ADDRESS1, L3GD20H_WHO_AM_I, 1, &result, "")) {
        if (result == 0xd7) {
            imuType = RTIMU_TYPE_GD20M303;
            slaveAddress = L3GD20H_ADDRESS1;
            I2CClose();
            HAL_INFO("Detected L3GD20H at option address\n");
            return true;
        }
    }

    I2CClose();

    HAL_ERROR("No IMU detected\n");
    return false;
}

bool RTIMUSettings::loadSettings()
{
    char buf[200];
    char key[200];
    char val[200];
    RTFLOAT ftemp;

    //  preset general defaults

    m_imuType = RTIMU_TYPE_AUTODISCOVER;
    m_I2CSlaveAddress = 0;
    m_I2CBus = 1;
    m_fusionType = RTFUSION_TYPE_KALMANSTATE4;
    m_compassCalValid = false;

    //  MPU9150 defaults

    m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;

    //  GD20M303 defaults

    m_GD20M303GyroSampleRate = L3GD20H_SAMPLERATE_50;
    m_GD20M303AccelSampleRate = 0;
    m_GD20M303CompassSampleRate = 0;
    m_GD20M303GyroBW = L3GD20H_BANDWIDTH_1;
    m_GD20M303GyroHpf = L3GD20H_HPF_4;
    m_GD20M303GyroFsr = L3GD20H_FSR_500;

    //  check to see if settings file exists

    if (!(m_fd = fopen(m_filename, "r"))) {
        HAL_INFO("Settings file not found. Using defaults and creating settings file\n");
        return saveSettings();
    }

    while (fgets(buf, 200, m_fd)) {
        if (sscanf(buf, "%[^=]=%s", key, val) != 2) {
            HAL_ERROR1("Bad line in settings file: %s\n", buf);
            fclose(m_fd);
            return false;
        }

        //  now decode keys

        if (strcmp(key, RTIMULIB_IMU_TYPE) == 0) {
            m_imuType = atoi(val);
        } else if (strcmp(key, RTIMULIB_FUSION_TYPE) == 0) {
            m_fusionType = atoi(val);
        } else if (strcmp(key, RTIMULIB_I2C_BUS) == 0) {
            m_I2CBus = atoi(val);
        } else if (strcmp(key, RTIMULIB_I2C_SLAVEADDRESS) == 0) {
            m_I2CSlaveAddress = atoi(val);
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_VALID) == 0) {
            m_compassCalValid = strcmp(val, "true") == 0;
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_MINX) == 0) {
            sscanf(val, "%f", &ftemp);
            m_compassCalMin.setX(ftemp);
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_MINY) == 0) {
            sscanf(val, "%f", &ftemp);
            m_compassCalMin.setY(ftemp);
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_MINZ) == 0) {
            sscanf(val, "%f", &ftemp);
            m_compassCalMin.setZ(ftemp);
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_MAXX) == 0) {
            sscanf(val, "%f", &ftemp);
            m_compassCalMax.setX(ftemp);
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_MAXY) == 0) {
            sscanf(val, "%f", &ftemp);
            m_compassCalMax.setY(ftemp);
        } else if (strcmp(key, RTIMULIB_COMPASSCAL_MAXZ) == 0) {
            sscanf(val, "%f", &ftemp);
            m_compassCalMax.setZ(ftemp);

        //  MPU9150 settings

        } else if (strcmp(key, RTIMULIB_MPU9150_GYROACCEL_SAMPLERATE) == 0) {
            m_MPU9150GyroAccelSampleRate = atoi(val);
        } else if (strcmp(key, RTIMULIB_MPU9150_COMPASS_SAMPLERATE) == 0) {
            m_MPU9150CompassSampleRate = atoi(val);
        } else if (strcmp(key, RTIMULIB_MPU9150_GYROACCEL_LPF) == 0) {
            m_MPU9150GyroAccelLpf = atoi(val);
        } else if (strcmp(key, RTIMULIB_MPU9150_GYRO_FSR) == 0) {
            m_MPU9150GyroFsr = atoi(val);
        } else if (strcmp(key, RTIMULIB_MPU9150_ACCEL_FSR) == 0) {
            m_MPU9150AccelFsr = atoi(val);

        //  Handle unrecognized key

        } else {
            HAL_ERROR1("Unrecognized key in settings file: %s\n", buf);
        }
    }
    HAL_INFO1("Settings file %s loaded\n", m_filename);
    fclose(m_fd);
    return saveSettings();                                  // make sure settings file is correct and complete
}

bool RTIMUSettings::saveSettings()
{
    if (!(m_fd = fopen(m_filename, "w"))) {
        HAL_ERROR("Failed to open settings file for save");
        return false;
    }
    setValue(RTIMULIB_IMU_TYPE, m_imuType);
    setValue(RTIMULIB_FUSION_TYPE, m_fusionType);
    setValue(RTIMULIB_I2C_BUS, m_I2CBus);
    setValue(RTIMULIB_I2C_SLAVEADDRESS, m_I2CSlaveAddress);

    setValue(RTIMULIB_COMPASSCAL_VALID, m_compassCalValid);
    setValue(RTIMULIB_COMPASSCAL_MINX, m_compassCalMin.x());
    setValue(RTIMULIB_COMPASSCAL_MINY, m_compassCalMin.y());
    setValue(RTIMULIB_COMPASSCAL_MINZ, m_compassCalMin.z());
    setValue(RTIMULIB_COMPASSCAL_MAXX, m_compassCalMax.x());
    setValue(RTIMULIB_COMPASSCAL_MAXY, m_compassCalMax.y());
    setValue(RTIMULIB_COMPASSCAL_MAXZ, m_compassCalMax.z());

    setValue(RTIMULIB_MPU9150_GYROACCEL_SAMPLERATE, m_MPU9150GyroAccelSampleRate);
    setValue(RTIMULIB_MPU9150_COMPASS_SAMPLERATE, m_MPU9150CompassSampleRate);
    setValue(RTIMULIB_MPU9150_GYROACCEL_LPF, m_MPU9150GyroAccelLpf);
    setValue(RTIMULIB_MPU9150_GYRO_FSR, m_MPU9150GyroFsr);
    setValue(RTIMULIB_MPU9150_ACCEL_FSR, m_MPU9150AccelFsr);
    fclose(m_fd);
    return true;
}

void RTIMUSettings::setValue(const char *key, const bool val)
{
    fprintf(m_fd, "%s=%s\n", key, val ? "true" : "false");
}

void RTIMUSettings::setValue(const char *key, const int val)
{
    fprintf(m_fd, "%s=%d\n", key, val);
}

void RTIMUSettings::setValue(const char *key, const RTFLOAT val)
{
    fprintf(m_fd, "%s=%f\n", key, val);
}


