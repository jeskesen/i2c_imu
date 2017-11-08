i2c_imu
=======

ROS driver for several 9-DOF IMUs.  This node is a ROS interface for [RTIMULib2](https://github.com/RTIMULib/RTIMULib2) by [richards-tech](http://richardstechnotes.wordpress.com/), a C++ library of drivers & filters of operating I2C IMUs in linux.  I take no credit for RTIMULib (the hard part), I simply created a ROS interface to it (the easy part).

The list of IMUs supported by RTIMULib is as follows:

- [MPU-9150 by Invensense](http://www.invensense.com/mems/gyro/mpu9150.html)
- [MPU-9250 by Invensense](http://www.invensense.com/mems/gyro/mpu9250.html)
- [LSM9DS0 by STMicroelectronics](http://www.st.com/web/en/catalog/sense_power/FM89/SC1448/PF258556)
- [Adafruit 9-DOF IMU Breakout - L3GD20 + LSM303](http://www.adafruit.com/product/1714)
- [MinIMU-9 v3 Gyro, Accelerometer, and Compass (L3GD20H and LSM303D Carrier) by Polulu](http://www.pololu.com/product/2468)


