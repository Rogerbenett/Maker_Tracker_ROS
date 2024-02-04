#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "aruco.h"



using namespace std;
using namespace cv;

cv::Mat image;

aruco::CameraParameters CamParam;

void camCallback(const sensor_msgs::CompressedImage& msg) {
    
    // cv_bridge::cv_bridge bridge;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    image = cv_ptr->image;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
    
    if (!camera_info_msg) {
        ROS_ERROR("Failed to receive camera_info message");
        return;
    }

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F);
    cameraMatrix.at<float>(0,0)=camera_info_msg->K[0];
    cameraMatrix.at<float>(0,2)=camera_info_msg->K[2];
    cameraMatrix.at<float>(1,1)=camera_info_msg->K[4];
    cameraMatrix.at<float>(1,2)=camera_info_msg->K[5];
    cameraMatrix.at<float>(2,2)=camera_info_msg->K[8];
    cv::Mat distortionCoeffs = cv::Mat(1, 4, CV_32F);
    cameraMatrix.at<float>(0,0)=camera_info_msg->D[0];
    cameraMatrix.at<float>(0,1)=camera_info_msg->D[1];
    cameraMatrix.at<float>(0,2)=camera_info_msg->D[2];
    cameraMatrix.at<float>(0,3)=camera_info_msg->D[3];
    cv::Size imageSize(camera_info_msg->width, camera_info_msg->height);

    // cv::Mat undistortedImage;
    // cv::undistort(image, undistortedImage, cameraMatrix, distortionCoeffs);

    // aruco::CameraParameters CamParam(cameraMatrix, distortionCoeffs, imageSize);

    // cv::imshow("Undistorted Image", undistortedImage);
    CamParam.setParams(cameraMatrix, distortionCoeffs, imageSize);
    //cout<<camera_info_msg->D[0]<<endl;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "camera_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw/compressed", 10, camCallback);
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/camera_info", 10, cameraInfoCallback);

    //ros::spin();
    while(1)
    {
        ros::spinOnce();
        if(image.rows!=0)
        cv::imshow("Image", image);
        if(char(cv::waitKey(10))==27)
             break;
        
    }
    return 0;
}