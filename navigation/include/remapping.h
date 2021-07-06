#ifndef REMAPPING_H
#define REMAPPING_H


#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

#define TIME_STEP 32


class Remapping{
    private:
        ros::NodeHandle nh_;
        std::string robot_name_;
        ros::Subscriber subscribe_name_ , subscribe_laser_;
        ros::Publisher publish_laser_;
        sensor_msgs::LaserScan local_scan_msg_;

    public:
        Remapping(ros::NodeHandle* nodehandle);
        void NameCallBack(const std_msgs::String& msg);
        void LaserCallBack(const sensor_msgs::LaserScan& msg);
};

#endif