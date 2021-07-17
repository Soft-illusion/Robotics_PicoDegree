#include <sensor_enable.h>

SensorEnable::SensorEnable(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    srv_timestep.request.value = TIME_STEP;
    srv_inf.request.value = INFINITY;
    srv_zero.request.value = 0.0;

    std::cout<<srv_timestep.request.value<<std::endl;
    subscribe_name_ = nh_.subscribe("/model_name", 1, &SensorEnable::NameCallBack,this);
    subscribe_cmd_vel_ = nh_.subscribe("/cmd_vel", 1, &SensorEnable::CmdvelCallBack,this);

    
}

void SensorEnable::CmdvelCallBack(const geometry_msgs::Twist& msg){
  linear_vel = msg.linear.x;
  angular_vel = msg.angular.z;
  srv_act.request.value = (linear_vel - angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
  vec_velocity_[0].call(srv_act);
  vec_velocity_[2].call(srv_act);
  srv_act.request.value = (linear_vel + angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
  vec_velocity_[1].call(srv_act);
  vec_velocity_[3].call(srv_act);
}

void SensorEnable::Initialize_sensors(){
    std::vector<std::string> sensors{"/CAM" , "/Lidar" ,"/ds_left" , "/ds_right" , "/keyboard" , "/global" , "/IMU" , "/Linear_sensor" , "/Rotation_sensor"};
    std::vector<ros::ServiceClient> vec_client; 
    for (auto sensor = sensors.begin(); sensor != sensors.end(); ++sensor){
        vec_client.push_back(nh_.serviceClient<webots_ros::set_int>(SensorEnable::robot_name_+ *sensor +"/enable"));
        ros::service::waitForService(SensorEnable::robot_name_ + *sensor + "/enable");
        vec_client.back().call(srv_timestep);
    }
    subscribe_keyboard_ = nh_.subscribe(SensorEnable::robot_name_+"/keyboard/key", 1, &SensorEnable::KeyboardCallBack,this);
    std::vector<std::string> actuators{"/wheel1" , "/wheel2" ,"/wheel3" , "/wheel4" , "/linear" , "/RM"};
    
    for (auto actuator = actuators.begin(); actuator != actuators.end(); ++actuator){
        vec_client.push_back(nh_.serviceClient<webots_ros::set_float>(SensorEnable::robot_name_+ *actuator +"/set_position"));
        ros::service::waitForService(SensorEnable::robot_name_ + *actuator + "/set_position");
        vec_client.back().call(srv_inf);

        vec_velocity_.push_back(nh_.serviceClient<webots_ros::set_float>(SensorEnable::robot_name_+ *actuator +"/set_velocity"));
        ros::service::waitForService(SensorEnable::robot_name_ + *actuator + "/set_velocity");
        vec_velocity_.back().call(srv_zero);
    }
}

void SensorEnable::NameCallBack(const std_msgs::String& msg){
    SensorEnable::robot_name_ = msg.data;
    Initialize_sensors();
}

void SensorEnable::KeyboardCallBack(const webots_ros::Int32Stamped& msg){
    teleop(msg.data);
}

void SensorEnable::teleop(int key){
    // UP 315
    // DOWN 317
    // LEFT 314
    // RIGTH 316
    // W 87
    // A 65
    // S 83
    // D 68
    switch(key) {
      case 87 :
        srv_act.request.value = 0.1;
        vec_velocity_[4].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[5].call(srv_act);
        break;

      case 83 :
        srv_act.request.value = -0.1;
        vec_velocity_[4].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[5].call(srv_act);
        break;

      case 65 :
        srv_act.request.value = -0.4;
        vec_velocity_[5].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        break;

      case 68 :
        srv_act.request.value = 0.4;
        vec_velocity_[5].call(srv_act);
        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        break;

      case 315 :
        srv_act.request.value = 2.0;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[1].call(srv_act);
        vec_velocity_[2].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;

      case 317 :
        srv_act.request.value = -2.0;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[1].call(srv_act);
        vec_velocity_[2].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;

      case 316 :
        srv_act.request.value = 1;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[2].call(srv_act);
        srv_act.request.value = -1;
        vec_velocity_[1].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;

      case 314 :
        srv_act.request.value = -1;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[2].call(srv_act);
        srv_act.request.value = 1;
        vec_velocity_[1].call(srv_act);
        vec_velocity_[3].call(srv_act);

        srv_act.request.value = 0;
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
        break;

      default :
        srv_act.request.value = 0;
        vec_velocity_[0].call(srv_act);
        vec_velocity_[1].call(srv_act);
        vec_velocity_[2].call(srv_act);
        vec_velocity_[3].call(srv_act);
        vec_velocity_[4].call(srv_act);
        vec_velocity_[5].call(srv_act);
   }
    std::cout<<key<<std::endl;

}
 
int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_enable");
    ros::NodeHandle nh;
    SensorEnable member(&nh);
    ros::spin();

    return 0;
}