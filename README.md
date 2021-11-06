# i2c_imu

ROS driver for several 9-DOF IMUs.  This node is a ROS interface for [RTIMULib2](https://github.com/RTIMULib/RTIMULib2) by [richards-tech](http://richardstechnotes.wordpress.com/), a C++ library of drivers & filters of operating I2C IMUs in linux.  I take no credit for RTIMULib (the hard part), I simply created a ROS interface to it (the easy part).

The list of IMUs supported by RTIMULib is as follows:

- [MPU-9150 by Invensense](http://www.invensense.com/mems/gyro/mpu9150.html)
- [MPU-9250 by Invensense](http://www.invensense.com/mems/gyro/mpu9250.html)
- [LSM9DS0 by STMicroelectronics](http://www.st.com/web/en/catalog/sense_power/FM89/SC1448/PF258556)
- [Adafruit 9-DOF IMU Breakout - L3GD20 + LSM303](http://www.adafruit.com/product/1714)
- [MinIMU-9 v3 Gyro, Accelerometer, and Compass (L3GD20H and LSM303D Carrier) by Polulu](http://www.pololu.com/product/2468)

## Installation
1. Install RTIMULib2 dependencies. Qt4 is necessary to build the GUI apps included in the library and Octave is used for ellipsoid calibration of the magnetometer.
    ```bash
    sudo apt update
    sudo apt install qt4-default liboctave-dev
    ```

2. Build and install RTIMULib2 from source
    ```bash
    mkdir ~/software && cd ~/software
    git clone https://github.com/RTIMULib/RTIMULib2.git
    cd RTIMULib2/Linux
    mkdir build && cd build
    cmake ..
    make -j4
    sudo make install
    ```

3. Build the package
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

## Usage
The simplest way to use this package is with
```bash
roslaunch i2c_imu i2c_imu_auto.launch
```
This will automatically detect the IMU and use the default parameters.

If you want to use custom parameters, such as calibration data, you can specify them directly in the launch file or in a yaml file. Look at `mpu_9150_param.launch` or `mpu_9150.launch` for an example.

## How to calibrate
Calibration can be done using the tools from RTIMULib2. Intructions on how to do so can be found in [this guide](https://github.com/RTIMULib/RTIMULib2/blob/master/Calibration.pdf)

