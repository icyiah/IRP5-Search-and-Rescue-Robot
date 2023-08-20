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

#include <nav_msgs/Odometry.h>

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


#include <unistd.h>

ImageProcessor::ImageProcessor(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_image_ = nh_.subscribe("/car3/camera/color/image_raw", 1, &ImageProcessor::ImageCallBack,this);
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
        if (msg.bounding_boxes[i].Class == "person" && msg.bounding_boxes[i].probability > 0.7 ){
            std::ostringstream path;
            path << ros::package::getPath("darknet_ros") << "/images/human" << num_ <<".png";

            cv::imwrite(path.str(),cv_ptr_->image);
            num_++;
            usleep(1000000); //Sleep for 1000ms
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

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("pos: [%f]", msg->pose.pose.position.x);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "Image_process");
    ros::NodeHandle nh;
    ImageProcessor member(&nh);
    ros::spin();
    
    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);

    return 0;
}
