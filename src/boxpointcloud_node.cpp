
// ##############################################
// #
// #Purpose: Subscribe to kinect depth point cloud ==> box detect
// #Author: Moonyoung Lee (ml634@kaist.ac.kr)
// #Date:  03.2018
// #
// ##############################################


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>


ros::Publisher pub;

//global var for box pos, width,height
int boxPixelX = 350;
int boxPixelY = 306;
int boxPixelWidth = 110;
int boxPixelHeight = 133;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

 



}



int main(int argc, char **argv)
{

    //initialize node boxDetect
    ros::init(argc,argv, "boxDetect_PCL_node");
    std::cout << "Starting boxDetect_PCL_node" << std::endl;

    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();

}

