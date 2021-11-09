# i2c_imu

ROS driver for several 9-DOF IMUs.  This node is a ROS interface for [RTIMULib2](https://github.com/RTIMULib/RTIMULib2) by [richards-tech](http://richardstechnotes.wordpress.com/), a C++ library of drivers & filters of operating I2C IMUs in linux.  I take no credit for RTIMULib (the hard part), I simply created a ROS interface to it (the easy part).

The list of IMUs supported by RTIMULib is as follows:

- [MPU-9150 by Invensense](http://www.invensense.com/mems/gyro/mpu9150.html)
- [MPU-9250 by Invensense](http://www.invensense.com/mems/gyro/mpu9250.html)
- [LSM9DS0 by STMicroelectronics](http://www.st.com/web/en/catalog/sense_power/FM89/SC1448/PF258556)
- [Adafruit 9-DOF IMU Breakout - L3GD20 + LSM303](http://www.adafruit.com/product/1714)
- [MinIMU-9 v3 Gyro, Accelerometer, and Compass (L3GD20H and LSM303D Carrier) by Polulu](http://www.pololu.com/product/2468)

## i2c_imu_node
### Published topics
* data (sensor_msgs/Imu): Data from the IMU.
* mag (sensor_msgs/MagneticField): Data from the magnetometer, only published if the `publish_magnetometer` parameter is set to true.
* euler (geometry_msgs/Vector3): Orientation as euler angles, only published if the `publish_euler` parameter is set to true.

### Parameters
* ~frame_id (string, default: "imu_link"): Name of the IMU's frame.
* ~publish_magnetometer (bool, default: false): Flag to publish the magnetometer data as a sensor_msgs/MagneticField message.
* ~publish_euler (bool, default: false): Flag to publish the orientation data as euler angles in a geometry_msgs/Vector3 message.
* ~orientation_covariance (double[9], default: [0] * 9): Orientation covariance matrix.
* ~angular_velocity_covariance (double[9], default: [0] * 9): Angular velocity covariance matrix.
* ~linear_acceleration_covariance (double[9], default: [0] * 9): Linear acceleration covariance matrix.
* ~magnetic_declination (double, default: 0): Magnetic declaration. You can find it using a [magnetic declination estimator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).

**IMU specific parameters are not listed here**

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
Calibration can be done using the tools from RTIMULib2. Instructions on how to do so can be found in [this guide](https://github.com/RTIMULib/RTIMULib2/blob/master/Calibration.pdf).

**Important**: to avoid an error while doing ellipsoid calibration, you'll want to run `RTIMULibCal` from the `RTEllipsoidFit` folder located where you cloned the RTIMULib2 repo. For example:
```bash
cd ~/dev/RTIMULib2/RTEllipsoidFit/
RTIMULibCal
```

By default, the calibration data will be written to a file named `RTIMULib.ini` located in the directory where you ran the calibration tool. In order for the `i2c_imu_node` to use this data, it needs to be converted into a yaml file. A script has been made to automatically do the conversion. It can be found in the `scripts` folder.

**Note**: right now, it will only convert the data for the mpu9250 imu.

### Usage
```bash
cd scripts
./convert_params path/to/RTIMULib.ini path/to/params.yaml
```


