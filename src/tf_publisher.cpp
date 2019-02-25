
// ##############################################
// #
// #Purpose: Subscribe to kinect depth point cloud ==> box detect
// #Author: Moonyoung Lee (ml634@kaist.ac.kr)
// #Date:  07.2018
// #
// ##############################################


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/Imu.h"

#include <tf/transform_listener.h>

#include <fstream>

ros::Publisher pub;
geometry_msgs::Pose tfPose;

using namespace std;
ofstream outputFile;
float qIMUrawX;
float qIMUrawY;
float qIMUrawZ;
float qIMUrawW;

tf::StampedTransform transformIMU;

float yawIMU;
float yawDegreeIMU;
	  
/*global var*/
ros::Time currentTime, beginTime;
int FPScount = 0;

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  qIMUrawX =msg->orientation.x;
  qIMUrawY =msg->orientation.y;
  qIMUrawZ =msg->orientation.z;
  qIMUrawW =msg->orientation.w;
  
  tf::Quaternion qIMU(qIMUrawX,qIMUrawY,qIMUrawZ,qIMUrawW);
  yawIMU = tf::getYaw(qIMU);
  yawDegreeIMU = yawIMU*180/3.14;
  
  
}


int main(int argc, char **argv)
{

	outputFile.open("tfOutput.csv");
	outputFile << "X" << "," <<  "Y" << "," <<  "Z" << "," << "yaw_robot" << "," << "yaw_imu" << "," <<std::endl;
	
    //initialize node boxDetect
    ros::init(argc,argv, "tf_publisher");

    ros::NodeHandle nh;
    

    // Create a ROS publisher for pose
    pub = nh.advertise<geometry_msgs::Pose>("mobile_hubo/tf_pose", 10);
	
	tf::TransformListener listener;


    ros::Rate loop_rate(100);
    
    ros::Subscriber sub = nh.subscribe("/imu", 1000, chatterCallback);


  while (ros::ok())
  {
	  tf::StampedTransform transform;
	  tf::Quaternion q;
	  float yaw;
	  float yawDegree;
	  
	  try{
		
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

	  }
	  catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
		}
  
  
  
	

	tfPose.position.x=transform.getOrigin().x();
	tfPose.position.y=transform.getOrigin().y();
	tfPose.position.z=transform.getOrigin().z();
	q = transform.getRotation();
	yaw = tf::getYaw(q);
	yawDegree = yaw*180/3.14;
	
	
	
	if (tfPose.position.x > 0.000001 || tfPose.position.x < -0.000001) 
	{ 
		
		//determine FPS
		currentTime = ros::Time::now();
		ros::Duration durationTime = currentTime - beginTime;
		FPScount++;
		if (durationTime.toSec() > 1.0) {
			ROS_INFO("FPS of lidar,camera,imu sync: %d\n", FPScount);
			FPScount = 0;
			beginTime = currentTime;

		}
	
		//ROS_INFO("x: %f, y: %f, z: %f\n",tfPose.position.x,tfPose.position.y,tfPose.position.z);
		//outputFile << tfPose.position.x << "," <<  tfPose.position.y << "," <<  tfPose.position.z << "," << std::endl;
		ROS_INFO("x: %.3f, y: %.3f, z: %.3f yaw_robot: %.2f, yaw_imu: %2.f\n",tfPose.position.x,tfPose.position.y,tfPose.position.z, yawDegree,yawDegreeIMU); 
		outputFile << tfPose.position.x << "," <<  tfPose.position.y << "," <<  tfPose.position.z << "," << yawDegree<< ","<<  yawDegreeIMU<<  ","<<std::endl;
	}

    pub.publish(tfPose);
    
    
    


    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

