#include <remapping.h>

Remapping::Remapping(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_name_ = nh_.subscribe("/model_name", 1, &Remapping::NameCallBack,this);
    publish_laser_ = nh_.advertise<sensor_msgs::LaserScan>("/LaserScan", 1000);
    publish_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("/PointCloud", 1000);

}



void Remapping::NameCallBack(const std_msgs::String& msg){
    Remapping::robot_name_ = msg.data;
    subscribe_laser_ = nh_.subscribe(robot_name_+"/Lidar/laser_scan/layer0", 1, &Remapping::LaserCallBack,this);
    subscribe_point_cloud_ = nh_.subscribe(robot_name_+"/Lidar/point_cloud", 1, &Remapping::PointCloudCallBack,this);

}



void Remapping::LaserCallBack(const sensor_msgs::LaserScan& msg){
    local_scan_msg_ = msg;
    // local_scan_msg_.header.frame_id = "base_link";
    publish_laser_.publish(local_scan_msg_);
}

void Remapping::PointCloudCallBack(const sensor_msgs::PointCloud& msg){
    local_point_msg_ = msg;
    local_point_msg_.header.frame_id = "point_cloud_link";
    publish_point_cloud_.publish(local_point_msg_);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Remapping");
    ros::NodeHandle nh;
    Remapping member(&nh);
    ros::spin();

    return 0;
}