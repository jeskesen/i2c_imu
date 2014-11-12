//  Copyright (c) 2014 Justin Eskesen
//
//  This file is part of i2c_imu
//
//  i2c_imu is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  i2c_imu is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with i2c_imu.  If not, see <http://www.gnu.org/licenses/>.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include "RTIMU.h"
#include "RTIMUSettings.h"

#define G_2_MPSS 9.80665

class I2cImu
{
public:
    I2cImu();

    void update();
    void spin();

private:
    //ROS Stuff
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher imu_pub_;
    ros::Publisher* magnetometer_pub_;

    std::string imu_frame_id_;

    ros::Time last_update_;

    //RTUIMULib stuff
    RTIMUSettings imu_settings_;
    RTIMU *imu_;
};

I2cImu::I2cImu() : nh_(), private_nh_("~")
{

	// params used for all
    private_nh_.param<std::string>("frame_id", imu_frame_id_, "imu_link");

    private_nh_.param<int>("imu_type", imu_settings_.m_imuType, imu_settings_.m_imuType);

    private_nh_.param<int>("fusion_type", imu_settings_.m_fusionType, imu_settings_.m_fusionType);
    int temp_int;
    private_nh_.param<int>("i2c_bus", temp_int, (int)imu_settings_.m_I2CBus);
    imu_settings_.m_I2CBus = (unsigned char)temp_int;

    private_nh_.param<int>("i2c_slave_address", temp_int, (int)imu_settings_.m_I2CSlaveAddress);
    imu_settings_.m_I2CSlaveAddress = (unsigned char)temp_int;

    //MPU9150
    private_nh_.param<int>("mpu9150/gyro_accel_sample_rate", imu_settings_.m_MPU9150GyroAccelSampleRate, imu_settings_.m_MPU9150GyroAccelSampleRate);
    private_nh_.param<int>("mpu9150/compass_sample_rate", imu_settings_.m_MPU9150CompassSampleRate, imu_settings_.m_MPU9150CompassSampleRate);
    private_nh_.param<int>("mpu9150/accel_full_scale_range", imu_settings_.m_MPU9150AccelFsr, imu_settings_.m_MPU9150AccelFsr);
    private_nh_.param<int>("mpu9150/gyro_accel_low_pass_filter", imu_settings_.m_MPU9150GyroAccelLpf, imu_settings_.m_MPU9150GyroAccelLpf);
    private_nh_.param<int>("mpu9150/gyro_full_scale_range", imu_settings_.m_MPU9150GyroFsr, imu_settings_.m_MPU9150GyroFsr);

    //MPU9250
    private_nh_.param<int>("mpu9250/gyro_accel_sample_rate", imu_settings_.m_MPU9250GyroAccelSampleRate, imu_settings_.m_MPU9250GyroAccelSampleRate);
    private_nh_.param<int>("mpu9250/compass_sample_rate", imu_settings_.m_MPU9250CompassSampleRate, imu_settings_.m_MPU9250CompassSampleRate);
    private_nh_.param<int>("mpu9250/accel_full_scale_range", imu_settings_.m_MPU9250AccelFsr, imu_settings_.m_MPU9250AccelFsr);
    private_nh_.param<int>("mpu9250/accel_low_pass_filter", imu_settings_.m_MPU9250AccelLpf, imu_settings_.m_MPU9250AccelLpf);
    private_nh_.param<int>("mpu9250/gyro_full_scale_range", imu_settings_.m_MPU9250GyroFsr, imu_settings_.m_MPU9250GyroFsr);
    private_nh_.param<int>("mpu9250/gyro_low_pass_filter", imu_settings_.m_MPU9250GyroLpf, imu_settings_.m_MPU9250GyroLpf);

    //GD20HM303D
    private_nh_.param<int>("GD20HM303D/gyro_sample_rate", imu_settings_.m_GD20HM303DGyroSampleRate, imu_settings_.m_GD20HM303DGyroSampleRate);
    private_nh_.param<int>("GD20HM303D/accel_sample_rate", imu_settings_.m_GD20HM303DAccelSampleRate, imu_settings_.m_GD20HM303DAccelSampleRate);
    private_nh_.param<int>("GD20HM303D/compass_sample_rate", imu_settings_.m_GD20HM303DCompassSampleRate, imu_settings_.m_GD20HM303DCompassSampleRate);
    private_nh_.param<int>("GD20HM303D/accel_full_scale_range", imu_settings_.m_GD20HM303DAccelFsr, imu_settings_.m_GD20HM303DAccelFsr);
    private_nh_.param<int>("GD20HM303D/gyro_full_scale_range", imu_settings_.m_GD20HM303DGyroFsr, imu_settings_.m_GD20HM303DGyroFsr);
    private_nh_.param<int>("GD20HM303D/compass_full_scale_range", imu_settings_.m_GD20HM303DCompassFsr, imu_settings_.m_GD20HM303DCompassFsr);
    private_nh_.param<int>("GD20HM303D/accel_low_pass_filter", imu_settings_.m_GD20HM303DAccelLpf, imu_settings_.m_GD20HM303DAccelLpf);
    private_nh_.param<int>("GD20HM303D/gyro_high_pass_filter", imu_settings_.m_GD20HM303DGyroHpf, imu_settings_.m_GD20HM303DGyroHpf);
    private_nh_.param<int>("GD20HM303D/gyro_bandwidth", imu_settings_.m_GD20HM303DGyroBW, imu_settings_.m_GD20HM303DGyroBW);

    //GD20M303DLHC
    private_nh_.param<int>("GD20M303DLHC/gyro_sample_rate", imu_settings_.m_GD20M303DLHCGyroSampleRate, imu_settings_.m_GD20M303DLHCGyroSampleRate);
    private_nh_.param<int>("GD20M303DLHC/accel_sample_rate", imu_settings_.m_GD20M303DLHCAccelSampleRate, imu_settings_.m_GD20M303DLHCAccelSampleRate);
    private_nh_.param<int>("GD20M303DLHC/compass_sample_rate", imu_settings_.m_GD20M303DLHCCompassSampleRate, imu_settings_.m_GD20M303DLHCCompassSampleRate);
    private_nh_.param<int>("GD20M303DLHC/accel_full_scale_range", imu_settings_.m_GD20M303DLHCAccelFsr, imu_settings_.m_GD20M303DLHCAccelFsr);
    private_nh_.param<int>("GD20M303DLHC/gyro_full_scale_range", imu_settings_.m_GD20M303DLHCGyroFsr, imu_settings_.m_GD20M303DLHCGyroFsr);
    private_nh_.param<int>("GD20M303DLHC/compass_full_scale_range", imu_settings_.m_GD20M303DLHCCompassFsr, imu_settings_.m_GD20M303DLHCCompassFsr);
    private_nh_.param<int>("GD20M303DLHC/gyro_high_pass_filter", imu_settings_.m_GD20M303DLHCGyroHpf, imu_settings_.m_GD20M303DLHCGyroHpf);
    private_nh_.param<int>("GD20M303DLHC/gyro_bandwidth", imu_settings_.m_GD20M303DLHCGyroBW, imu_settings_.m_GD20M303DLHCGyroBW);


    //LSM9DS0
    private_nh_.param<int>("LSM9DS0/gyro_sample_rate", imu_settings_.m_LSM9DS0GyroSampleRate, imu_settings_.m_LSM9DS0GyroSampleRate);
    private_nh_.param<int>("LSM9DS0/accel_sample_rate", imu_settings_.m_LSM9DS0AccelSampleRate, imu_settings_.m_LSM9DS0AccelSampleRate);
    private_nh_.param<int>("LSM9DS0/compass_sample_rate", imu_settings_.m_LSM9DS0CompassSampleRate, imu_settings_.m_LSM9DS0CompassSampleRate);
    private_nh_.param<int>("LSM9DS0/accel_full_scale_range", imu_settings_.m_LSM9DS0AccelFsr, imu_settings_.m_LSM9DS0AccelFsr);
    private_nh_.param<int>("LSM9DS0/gyro_full_scale_range", imu_settings_.m_LSM9DS0GyroFsr, imu_settings_.m_LSM9DS0GyroFsr);
    private_nh_.param<int>("LSM9DS0/compass_full_scale_range", imu_settings_.m_LSM9DS0CompassFsr, imu_settings_.m_LSM9DS0CompassFsr);
    private_nh_.param<int>("LSM9DS0/accel_low_pass_filter", imu_settings_.m_LSM9DS0AccelLpf, imu_settings_.m_LSM9DS0AccelLpf);
    private_nh_.param<int>("LSM9DS0/gyro_high_pass_filter", imu_settings_.m_LSM9DS0GyroHpf, imu_settings_.m_LSM9DS0GyroHpf);
    private_nh_.param<int>("LSM9DS0/gyro_bandwidth", imu_settings_.m_LSM9DS0GyroBW, imu_settings_.m_LSM9DS0GyroBW);

    imu_ = RTIMU::createIMU(&imu_settings_);
    if(imu_==NULL)
    {
        ROS_FATAL("I2cImu - %s - Failed to open the i2c device", __FUNCTION__);
        ROS_BREAK();
    }

    imu_->IMUInit();
 
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1.0/(imu_->IMUGetPollInterval()/1000.0));

    bool magnetometer;
	private_nh_.param("publish_magnetometer", magnetometer, false);
	if(magnetometer)
	{
		magnetometer_pub_ = new ros::Publisher();
		*magnetometer_pub_ = nh_.advertise<geometry_msgs::Vector3>("/mag", 10, false);
	}
}

void I2cImu::update()
{

    while (imu_->IMURead())
    {
        RTIMU_DATA imuData = imu_->getIMUData();

        ros::Time current_time = ros::Time::now();
	// sensor msg topic output
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = imu_frame_id_;
        imu_msg.orientation.x = imuData.fusionQPose.x();
        imu_msg.orientation.y = imuData.fusionQPose.y();
        imu_msg.orientation.z = imuData.fusionQPose.z();
        imu_msg.orientation.w = imuData.fusionQPose.scalar();

        imu_msg.angular_velocity.x = imuData.gyro.x();
        imu_msg.angular_velocity.y = imuData.gyro.y();
        imu_msg.angular_velocity.z = imuData.gyro.z();

        imu_msg.linear_acceleration.x = imuData.accel.x() * G_2_MPSS;
        imu_msg.linear_acceleration.y = imuData.accel.y() * G_2_MPSS;
        imu_msg.linear_acceleration.z = imuData.accel.z() * G_2_MPSS;

        imu_pub_.publish(imu_msg);

        if(magnetometer_pub_ != NULL && imuData.compassValid)
        {
			 geometry_msgs::Vector3Stamped msg;
			 msg.header.stamp = current_time;
			 msg.header.frame_id = imu_frame_id_;
			 msg.vector.x = imuData.compass.x();
			 msg.vector.y = imuData.compass.y();
			 msg.vector.z = imuData.compass.z();

			 magnetometer_pub_->publish(msg);
         }

    }

}

void I2cImu::spin()
{
    ros::Rate r(1.0/(imu_->IMUGetPollInterval()/1000.0));
    while(nh_.ok())
    {
        update();

        ros::spinOnce();
        r.sleep();
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "i2c_imu_node");

    ROS_INFO("RTIMU Node for ROS");

    I2cImu i2c_imu;
    i2c_imu.spin();

    return(0);
}

// EOF
