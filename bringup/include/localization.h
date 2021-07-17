#ifndef LOCALIZATION_H
#define LOCALIZATION_H


#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define TIME_STEP 32


class Localization{
    private:
        ros::NodeHandle nh_;
        std::string robot_name_;
        ros::Subscriber subscribe_name_ , subscribe_gps_ , subscribe_imu_;
        std::vector<ros::ServiceClient> vec_velocity_;
        float current_x , current_y, current_z;
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;
        float roll_ , pitch_ , yaw_ ;


    public:
        Localization(ros::NodeHandle* nodehandle);
        void NameCallBack(const std_msgs::String& msg);
        void GPSCallBack(const geometry_msgs::PointStamped& msg);
        void IMUCallBack(const sensor_msgs::Imu& msg);
        void getIMU();
        void getGPS();
        void publish_base_link();
        void ToEulerAngles(float x, float y , float z , float w );
        void publish_lidar_link();

};

#endif