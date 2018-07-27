// ##############################################
// #
// #Purpose: Subscribe to kinect depth point cloud
// #Author: Moonyoung Lee (ml634@kaist.ac.kr)
// #Date:  03.2018
// #
// ##############################################


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV for image handling
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>



//global var for trackBar
int minH = 0, maxH = 26, minS = 58, maxS = 255, minV = 58, maxV = 255;
const char* windowName = "HSV tuner";


//call back function from subscriber
void boxImageHandler(const sensor_msgs::ImageConstPtr& msg)
{
    std::cout << "Received Img" << std::endl;
    //convert ROS image to OpenCV img pixel encoding

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

    //color change RGB2HSV
    cv::Mat HSVImage;
    cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);




    cv::Mat HSVThreshold;
    cv::inRange(HSVImage, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV),HSVThreshold);

    cv::imshow("view", HSVImage);
    cv::imshow(windowName, HSVThreshold);
    cv::waitKey(30);
}



int main(int argc, char **argv)
{

    //initialize node boxDetect
    ros::init(argc,argv, "boxDetect_node");
    std::cout << "Initialized boxDetect node" << std::endl;
    //start a node
    ros::NodeHandle nh;


    cv::namedWindow("view");
    cv::startWindowThread();

    //image sub/pub
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, boxImageHandler);


    //threshold HSV range for box

    cv::namedWindow(windowName);

    cv::createTrackbar("MinH", windowName, &minH, 180);
    cv::createTrackbar("MaxH", windowName, &maxH, 180);
    cv::createTrackbar("MinS", windowName, &minS, 255);
    cv::createTrackbar("MaxS", windowName, &maxS, 255);
    cv::createTrackbar("MinV", windowName, &minV, 255);
    cv::createTrackbar("MaxV", windowName, &maxV, 255);
/*

*/
    ros::spin();
    cv::destroyWindow("view");
    

}

