#include <Image_processor.h>
#include <unistd.h>

ImageProcessor::ImageProcessor(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_image_ = nh_.subscribe("/camera/rgb/image_raw", 1, &ImageProcessor::ImageCallBack,this);
    subscribe_result_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageProcessor::ResultCallBack,this);
    num_=0;

}

void ImageProcessor::ImageCallBack(const sensor_msgs::Image& msg){
    // std::cout<<"Image callback"<<std::endl;
    // Save current msg as a current image
    try{
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}

void ImageProcessor::ResultCallBack(const darknet_ros_msgs::BoundingBoxes& msg){
    // trigger saving of image
    for (int i=0 ; i<msg.bounding_boxes.size();i++){
        if (msg.bounding_boxes[i].Class == "person" && msg.bounding_boxes[i].probability > 0.5 ){
            std::ostringstream path;
            path << ros::package::getPath("bringup") << "/images/human" << num_ <<".png";

            cv::imwrite(path.str(),cv_ptr_->image);
            num_++;
            usleep(100000); //Sleep for 100ms

        }
    }

}



void ImageProcessor::ToEulerAngles(float x, float y , float z , float w ) {

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


int main(int argc, char **argv){
    ros::init(argc, argv, "Image_process");
    ros::NodeHandle nh;
    ImageProcessor member(&nh);
    ros::spin();

    return 0;
}
