#ifndef STATIC_CAMERA_H
#define STATIC_CAMERA_H


#include "ros/ros.h"
#include "webots_ros/Int32Stamped.h"
#include "webots_ros/Float64Stamped.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define TIME_STEP 32


class StaticCamera{
    private:
        ros::NodeHandle nh_;
        std::string robot_name_;
        ros::Publisher image_pub_;
        ros::Subscriber subscribe_name_ , subscribe_linear_ , subscribe_rotary_ , subscribe_image_;
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;
        float roll_ , pitch_ , yaw_ ;


    public:
        StaticCamera(ros::NodeHandle* nodehandle);
        void NameCallBack(const std_msgs::String& msg);
        void LinearCallBack(const webots_ros::Float64Stamped& msg);
        void RotaryCallBack(const webots_ros::Float64Stamped& msg);
        void ImageCallBack(const sensor_msgs::Image& msg);
        void getImage();
        void getLinear();
        void getRotary();
        void publish_linear_link(float value);
        void ToEulerAngles(float x, float y , float z , float w );
        void publish_camera_link(float value);

};

#endif