#!/usr/bin/env python3

import os
import sys
from pathlib import Path

source_path = sys.argv[1]
target_path = sys.argv[2]

# Make sure source path exists and create target directory if needed
if os.path.exists(source_path):
    print("Source path: '" + str(source_path) + "'")
else:
    print("Error: source path: '" + str(source_path) + "' does not exist")
    exit()

Path(os.path.dirname(target_path)).mkdir(parents=True, exist_ok=True)
print("Target path: '" + str(target_path) + "'")


imu_types = {
    2: "mpu9150",
    3: "GD20HM303D",
    4: "GD20M303DLHC",
    5: "LSM9DS0",
    7: "mpu9250",
    8: "GD20HM303DLHC"
}

imu_name = None
general_params = {}
imu_params = {}
calibration = {
    "compass_min": [0] * 3,
    "compass_max": [0] * 3,
    "compass_cal_offset": [0] * 3,
    "compass_cal_corr": [0] * 9,
    "accel_min": [0] * 3,
    "accel_max": [0] * 3
}


def get_value_from_line(line, key, dict, type, index=-1):
    line = line.strip()
    split_line = line.split("=")
    value = type(split_line[1])
    if index >= 0:
        dict[key][index] = value
    else:
        dict[key] = value


# Parse file
with open(source_path, "r") as f:
    lines = f.readlines()

    for line in lines:
        # Get general parameters
        if "IMUType" in line:
            get_value_from_line(line, "imu_type", general_params, int)
            imu_name = imu_types[general_params["imu_type"]]
        elif "I2CBus" in line:
            get_value_from_line(line, "i2c_bus", general_params, int)
        elif "I2CSlaveAddress" in line:
            get_value_from_line(line, "i2c_slave_address", general_params, int)
        elif "FusionType" in line:
            get_value_from_line(line, "fusion_type", general_params, int)
        elif "compassAdjDeclination" in line:
            get_value_from_line(line, "magnetic_declination", general_params, float)

        # Get calibration data
        elif "CompassCalMinX" in line:
            get_value_from_line(line, "compass_min", calibration, float, index=0)
        elif "CompassCalMinY" in line:
            get_value_from_line(line, "compass_min", calibration, float, index=1)
        elif "CompassCalMinZ" in line:
            get_value_from_line(line, "compass_min", calibration, float, index=2)
        elif "CompassCalMaxX" in line:
            get_value_from_line(line, "compass_max", calibration, float, index=0)
        elif "CompassCalMaxY" in line:
            get_value_from_line(line, "compass_max", calibration, float, index=1)
        elif "CompassCalMaxZ" in line:
            get_value_from_line(line, "compass_max", calibration, float, index=2)
        elif "compassCalOffsetX" in line:
            get_value_from_line(line, "compass_cal_offset", calibration, float, index=0)
        elif "compassCalOffsetY" in line:
            get_value_from_line(line, "compass_cal_offset", calibration, float, index=1)
        elif "compassCalOffsetZ" in line:
            get_value_from_line(line, "compass_cal_offset", calibration, float, index=2)
        elif "compassCalCorr" in line:
            split_line = line.split("=")
            row = int((split_line[0])[-2])
            col = int((split_line[0])[-1])
            index = (row - 1) * 3 + (col - 1)
            get_value_from_line(line, "compass_cal_corr", calibration, float, index=index)
        elif "AccelCalMinX" in line:
            get_value_from_line(line, "accel_min", calibration, float, index=0)
        elif "AccelCalMinY" in line:
            get_value_from_line(line, "accel_min", calibration, float, index=1)
        elif "AccelCalMinZ" in line:
            get_value_from_line(line, "accel_min", calibration, float, index=2)
        elif "AccelCalMaxX" in line:
            get_value_from_line(line, "accel_max", calibration, float, index=0)
        elif "AccelCalMaxY" in line:
            get_value_from_line(line, "accel_max", calibration, float, index=1)
        elif "AccelCalMaxZ" in line:
            get_value_from_line(line, "accel_max", calibration, float, index=2)

        # Get imu parameters
        if imu_name == "mpu9150":
            if "MPU9150GyroAccelSampleRate" in line:
                get_value_from_line(line, "gyro_accel_sample_rate", imu_params, int)
            elif "MPU9150CompassSampleRate" in line:
                get_value_from_line(line, "compass_sample_rate", imu_params, int)
            elif "MPU9150GyroAccelLpf" in line:
                get_value_from_line(line, "gyro_accel_low_pass_filter", imu_params, int)
            elif "MPU9150GyroFSR" in line:
                get_value_from_line(line, "gyro_full_scale_range", imu_params, int)
            elif "MPU9150AccelFSR" in line:
                get_value_from_line(line, "accel_full_scale_range", imu_params, int)
        elif imu_name == "mpu9250":
            if "MPU9250GyroAccelSampleRate" in line:
                get_value_from_line(line, "gyro_accel_sample_rate", imu_params, int)
            elif "MPU9250CompassSampleRate" in line:
                get_value_from_line(line, "compass_sample_rate", imu_params, int)
            elif "MPU9250GyroLpf" in line:
                get_value_from_line(line, "gyro_low_pass_filter", imu_params, int)
            elif "MPU9250AccelLpf" in line:
                get_value_from_line(line, "accel_low_pass_filter", imu_params, int)
            elif "MPU9250GyroFSR" in line:
                get_value_from_line(line, "gyro_full_scale_range", imu_params, int)
            elif "MPU9250AccelFSR" in line:
                get_value_from_line(line, "accel_full_scale_range", imu_params, int)
        elif imu_name == "GD20HM303D":
            if "GD20HM303DGyroSampleRate" in line:
                get_value_from_line(line, "gyro_sample_rate", imu_params, int)
            elif "GD20HM303DGyroFsr" in line:
                get_value_from_line(line, "gyro_full_scale_range", imu_params, int)
            elif "GD20HM303DGyroHpf" in line:
                get_value_from_line(line, "gyro_high_pass_filter", imu_params, int)
            elif "GD20HM303DGyroBW" in line:
                get_value_from_line(line, "gyro_bandwidth", imu_params, int)
            elif "GD20HM303DAccelSampleRate" in line:
                get_value_from_line(line, "accel_sample_rate", imu_params, int)
            elif "GD20HM303DAccelFsr" in line:
                get_value_from_line(line, "accel_full_scale_range", imu_params, int)
            elif "GD20HM303DAccelLpf" in line:
                get_value_from_line(line, "accel_low_pass_filter", imu_params, int)
            elif "GD20HM303DCompassSampleRate" in line:
                get_value_from_line(line, "compass_sample_rate", imu_params, int)
            elif "GD20HM303DCompassFsr" in line:
                get_value_from_line(line, "compass_full_scale_range", imu_params, int)
        elif imu_name == "GD20M303DLHC":
            if "GD20M303DLHCGyroSampleRate" in line:
                get_value_from_line(line, "gyro_sample_rate", imu_params, int)
            elif "GD20M303DLHCGyroFsr" in line:
                get_value_from_line(line, "gyro_full_scale_range", imu_params, int)
            elif "GD20M303DLHCGyroHpf" in line:
                get_value_from_line(line, "gyro_high_pass_filter", imu_params, int)
            elif "GD20M303DLHCGyroBW" in line:
                get_value_from_line(line, "gyro_bandwidth", imu_params, int)
            elif "GD20M303DLHCAccelSampleRate" in line:
                get_value_from_line(line, "accel_sample_rate", imu_params, int)
            elif "GD20M303DLHCAccelFsr" in line:
                get_value_from_line(line, "accel_full_scale_range", imu_params, int)
            elif "GD20M303DLHCCompassSampleRate" in line:
                get_value_from_line(line, "compass_sample_rate", imu_params, int)
            elif "GD20M303DLHCCompassFsr" in line:
                get_value_from_line(line, "compass_full_scale_range", imu_params, int)
        elif imu_name == "GD20HM303DLHC":
            if "GD20HM303DLHCGyroSampleRate" in line:
                get_value_from_line(line, "gyro_sample_rate", imu_params, int)
            elif "GD20HM303DLHCGyroFsr" in line:
                get_value_from_line(line, "gyro_full_scale_range", imu_params, int)
            elif "GD20HM303DLHCGyroHpf" in line:
                get_value_from_line(line, "gyro_high_pass_filter", imu_params, int)
            elif "GD20HM303DLHCGyroBW" in line:
                get_value_from_line(line, "gyro_bandwidth", imu_params, int)
            elif "GD20HM303DLHCAccelSampleRate" in line:
                get_value_from_line(line, "accel_sample_rate", imu_params, int)
            elif "GD20HM303DLHCAccelFsr" in line:
                get_value_from_line(line, "accel_full_scale_range", imu_params, int)
            elif "GD20HM303DLHCCompassSampleRate" in line:
                get_value_from_line(line, "compass_sample_rate", imu_params, int)
            elif "GD20HM303DLHCCompassFsr" in line:
                get_value_from_line(line, "compass_full_scale_range", imu_params, int)
        elif imu_name == "LSM9DS0":
            if "LSM9DS0GyroSampleRate" in line:
                get_value_from_line(line, "gyro_sample_rate", imu_params, int)
            elif "LSM9DS0GyroFsr" in line:
                get_value_from_line(line, "gyro_full_scale_range", imu_params, int)
            elif "LSM9DS0GyroHpf" in line:
                get_value_from_line(line, "gyro_high_pass_filter", imu_params, int)
            elif "LSM9DS0GyroBW" in line:
                get_value_from_line(line, "gyro_bandwidth", imu_params, int)
            elif "LSM9DS0AccelSampleRate" in line:
                get_value_from_line(line, "accel_sample_rate", imu_params, int)
            elif "LSM9DS0AccelFsr" in line:
                get_value_from_line(line, "accel_full_scale_range", imu_params, int)
            elif "LSM9DS0AccelLpf" in line:
                get_value_from_line(line, "accel_low_pass_filter", imu_params, int)
            elif "LSM9DS0CompassSampleRate" in line:
                get_value_from_line(line, "compass_sample_rate", imu_params, int)
            elif "LSM9DS0CompassFsr" in line:
                get_value_from_line(line, "compass_full_scale_range", imu_params, int)


# Write yaml file
with open(target_path, "w") as f:
    # General parameters
    general_param_keys = list(general_params.keys())
    for param in general_param_keys:
        f.write(param + ": " + str(general_params[param]) + "\n")

    # IMU parameters
    f.write("\n" + imu_name + ": \n")
    imu_param_keys = list(imu_params.keys())
    for param in imu_param_keys:
        f.write("  " + param + ": " + str(imu_params[param]) + "\n")

    # Calibration data
    f.write("\ncalib: \n")
    calibration_keys = list(calibration.keys())
    for key in calibration_keys:
        f.write("  " + key + ": " + str(calibration[key]) + "\n")

print("Conversion completed successfully")
