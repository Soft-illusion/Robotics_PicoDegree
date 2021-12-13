#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H


#include "ros/ros.h"

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

 // Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/package.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

class ImageProcessor{
    private:
        ros::NodeHandle nh_;
        std::string robot_name_;
        ros::Subscriber subscribe_image_ , subscribe_result_;
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;
        float roll_ , pitch_ , yaw_ ;
        cv_bridge::CvImagePtr cv_ptr_;
        int num_;

    public:
        ImageProcessor(ros::NodeHandle* nodehandle);
        void ImageCallBack(const sensor_msgs::Image& msg);
        void ResultCallBack(const darknet_ros_msgs::BoundingBoxes& msg);
        void ToEulerAngles(float x, float y , float z , float w );

};

#endif