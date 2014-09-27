//
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
    std::string imu_frame_id_;

    ros::Time last_update_;

    //RTUIMULib stuff
    RTIMUSettings imu_settings_;
    RTIMU *imu_;
};

I2cImu::I2cImu() : nh_(), private_nh_("~")
{

    private_nh_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");

    private_nh_.param<int>("imu_type", imu_settings_.m_imuType, RTIMU_TYPE_AUTODISCOVER);

    private_nh_.param<int>("fusion_type", imu_settings_.m_fusionType, RTFUSION_TYPE_RTQF);
    int temp_int;
    private_nh_.param<int>("i2c_bus", temp_int, 1);
    imu_settings_.m_I2CBus = (unsigned char)temp_int;
    //private_nh_.param<int>("i2c_slave_address", imu_settings_.m_I2CSlaveAddress, 0);

    imu_ = RTIMU::createIMU(&imu_settings_);
    if(imu_==NULL)
    {
        ROS_FATAL("I2cImu - %s - Failed to open the i2c device", __FUNCTION__);
        ROS_BREAK();
    }

    imu_->IMUInit();
 
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 50);

}

void I2cImu::update()
{

    while (imu_->IMURead())
    {
        RTIMU_DATA imuData = imu_->getIMUData();


        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = ros::Time::now();
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
