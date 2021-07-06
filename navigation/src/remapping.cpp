#include <remapping.h>

Remapping::Remapping(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_name_ = nh_.subscribe("/model_name", 1, &Remapping::NameCallBack,this);
    publish_laser_ = nh_.advertise<sensor_msgs::LaserScan>("/LaserScan", 1000);
}



void Remapping::NameCallBack(const std_msgs::String& msg){
    Remapping::robot_name_ = msg.data;
    subscribe_laser_ = nh_.subscribe(robot_name_+"/Lidar/laser_scan/layer0", 1, &Remapping::LaserCallBack,this);
}



void Remapping::LaserCallBack(const sensor_msgs::LaserScan& msg){
    local_scan_msg_ = msg;
    // local_scan_msg_.header.frame_id = "base_link";
    publish_laser_.publish(local_scan_msg_);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "Remapping");
    ros::NodeHandle nh;
    Remapping member(&nh);
    ros::spin();

    return 0;
}