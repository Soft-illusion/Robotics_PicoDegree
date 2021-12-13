#include <static_camera.h>

StaticCamera::StaticCamera(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_name_ = nh_.subscribe("/model_name", 1, &StaticCamera::NameCallBack,this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",1000);

}

void StaticCamera::publish_camera_link(float value){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0, 0 ,value );
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rotary_link" , "camera_link"));
}

void StaticCamera::getLinear(){
    subscribe_linear_ = nh_.subscribe(robot_name_+"/Linear_sensor/value", 1, &StaticCamera::LinearCallBack,this);
}

void StaticCamera::getRotary(){
    subscribe_rotary_ = nh_.subscribe(robot_name_+"/Rotation_sensor/value", 1, &StaticCamera::RotaryCallBack,this);
}

void StaticCamera::getImage(){
    subscribe_image_ = nh_.subscribe(robot_name_+"/CAM/image", 1, &StaticCamera::ImageCallBack,this);
}

void StaticCamera::NameCallBack(const std_msgs::String& msg){
    StaticCamera::robot_name_ = msg.data;
    getLinear();
    getRotary();
    getImage();
}

void StaticCamera::ImageCallBack(const sensor_msgs::Image& msg){
    image_pub_.publish(msg);
}


void StaticCamera::LinearCallBack(const webots_ros::Float64Stamped& msg){
    publish_linear_link(msg.data);

}

void StaticCamera::RotaryCallBack(const webots_ros::Float64Stamped& msg){
    publish_camera_link(msg.data);
}

void StaticCamera::ToEulerAngles(float x, float y , float z , float w ) {

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch_ = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch_ = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);

}

void StaticCamera::publish_linear_link(float value){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,value+0.05/2)); //Robot height/2 offset for linear link
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  // transform.setRotation(tf::Quaternion(current_rot_x,current_rot_y,current_rot_z,current_rot_w));
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link" , "linear_link"));
}


int main(int argc, char **argv){
    ros::init(argc, argv, "static_camera");
    ros::NodeHandle nh;
    StaticCamera member(&nh);
    ros::spin();

    return 0;
}