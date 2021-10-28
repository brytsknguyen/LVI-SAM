/**
* This file is part of VIRALC.
* 
* Copyright (C) 2020 Thien-Minh Nguyen <thienminh.nguyen at ntu dot edu dot sg>,
* School of EEE
* Nanyang Technological Univertsity, Singapore
* 
* For more information please see <https://britsknguyen.github.io>.
* or <https://github.com/britsknguyen/VIRALC>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* VIRALC is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* VIRALC is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with VIRALC.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#include <stdio.h>
#include <vector>
#include <sys/stat.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace Eigen;

typedef Eigen::Quaterniond Quaternd;

ros::Subscriber imuSub;
ros::Publisher imuPub;

Matrix4d tf_Bold_Bnew;

void imuCb(const sensor_msgs::ImuConstPtr &msg)
{
    Vector3d w(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vector3d a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Quaternd q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    static Matrix3d R_Bold_Bnew = tf_Bold_Bnew.block<3, 3>(0, 0);
    static Matrix3d R_Bnew_Bold = R_Bold_Bnew.inverse();


    w = R_Bnew_Bold*w;
    a = R_Bnew_Bold*a;
    q = q*R_Bold_Bnew;

    sensor_msgs::Imu imu;
    imu.header = msg->header;

    imu.angular_velocity.x = w(0);
    imu.angular_velocity.y = w(1);
    imu.angular_velocity.z = w(2);

    imu.linear_acceleration.x = a(0);
    imu.linear_acceleration.y = a(1);
    imu.linear_acceleration.z = a(2);

    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();

    imuPub.publish(imu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imuMod");
    ros::NodeHandle nh("~");
    
    // IMU topic name
    std::string imu_topic;
    nh.param("imu_topic", imu_topic, std::string("/imu/imu"));

    // Transform
    vector<double> tf_Bold_Bnew_;
    nh.param("tf_Bold_Bnew", tf_Bold_Bnew_, {1, 0, 0, 0,
                                             0, 1, 0, 0,
                                             0, 0, 1, 0,
                                             0, 0, 0, 1});
    tf_Bold_Bnew = Matrix<double, 4, 4, RowMajor>(&tf_Bold_Bnew_[0]);

    // Pub and Sub
    imuSub = nh.subscribe(imu_topic, 100, imuCb);
    imuPub = nh.advertise<sensor_msgs::Imu>(imu_topic + string("_mod"), 100);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    ros::spin();
}
