#ifndef SENSOR_ENABLE_H
#define SENSOR_ENABLE_H


#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

#define TIME_STEP 32
#define WHEEL_BASE 0.1
#define WHEEL_RADIUS 0.06



class SensorEnable{
    private:
        ros::NodeHandle nh_;
        std::string robot_name_;
        ros::Subscriber subscribe_name_;
        ros::Subscriber subscribe_keyboard_;
        webots_ros::set_int srv_timestep;
        webots_ros::set_float srv_inf;
        webots_ros::set_float srv_zero;
        std::vector<ros::ServiceClient> vec_velocity_;
        webots_ros::set_float srv_act;
        ros::Subscriber subscribe_cmd_vel_;
        float linear_vel, angular_vel ;

    public:
        SensorEnable(ros::NodeHandle* nodehandle);
        void NameCallBack(const std_msgs::String& msg);
        void KeyboardCallBack(const webots_ros::Int32Stamped& msg);
        void CmdvelCallBack(const geometry_msgs::Twist& msg);
        void Initialize_sensors();
        void teleop(int);
};

#endif