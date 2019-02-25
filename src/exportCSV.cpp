
// ##############################################
// #
// #Purpose: save topic data to CSV
// #Author: Moonyoung Lee (ml634@kaist.ac.kr)
// #Date:  01.2019
// #
// ##############################################


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "sensor_msgs/Imu.h"

#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "tf/transform_datatypes.h"

#include <fstream>

#define R2D			5.729577951308232e1
ros::Publisher pub;
geometry_msgs::Pose tfPose;

using namespace std;
ofstream outputFile;

float roll_a;
float pitch_a;
float yaw_a;



float roll_b;
float pitch_b;
float yaw_b;

float roll_c;
float pitch_c;
float yaw_c;

float gx_ang_vel[3] = {};
float gx_lin_acc[3] = {};

float bmi_ang_vel[3] = {};
float bmi_lin_acc[3] = {};

float adis_ang_vel[3] = {};
float adis_lin_acc[3] = {0};

float box[2] = {};
float v_ref[2] = {};
	  
/*global var*/
ros::Time currentTime, beginTime;
int FPScount = 0;

void callback3DM (const geometry_msgs::Vector3Stamped& msg)
{
   roll_a = msg.vector.x * R2D;
   pitch_a = msg.vector.y * R2D;
   yaw_a = msg.vector.z * R2D;
  
  
}


void callback16480(const geometry_msgs::Pose& msg)
{
	/*
  roll_b = msg.position.x;
   pitch_b = msg.position.y;
   yaw_b = msg.position.z;
  */
 
  //modified on how mount orientation
  roll_b = -msg.position.y ;
  pitch_b = msg.position.x ;
  yaw_b = msg.position.z ;
}

void callbackD435i(const sensor_msgs::Imu::ConstPtr& msg)
{
    
   
   // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

	// the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    roll_c = roll * R2D;
    pitch_c = pitch * R2D;
    yaw_c = yaw * R2D;
    
    
    //ROS_INFO("r: %.3f, p: %.3f, y: %.3f\n", roll_c, pitch_c, yaw_c); 

    
  
  
}

void callback3DM_raw (const sensor_msgs::Imu::ConstPtr& msg)
{
   gx_ang_vel[0] = msg->angular_velocity.x;
   gx_ang_vel[1] = msg->angular_velocity.y;
   gx_ang_vel[2] = msg->angular_velocity.z;
   
   gx_lin_acc[0] = msg->linear_acceleration.x;
   gx_lin_acc[1] = msg->linear_acceleration.y;
   gx_lin_acc[2] = msg->linear_acceleration.z;
   
   //ROS_INFO("r: %.3f, p: %.3f, y: %.3f\n", gx_ang_vel[0], gx_ang_vel[1], gx_ang_vel[2]); 
  
}

void callback16480_raw(const geometry_msgs::Pose& msg)
{
	/*
  roll_b = msg.position.x;
   pitch_b = msg.position.y;
   yaw_b = msg.position.z;
  */
 
  //modified on how mount orientation
  adis_ang_vel[0] = -msg.position.y ;
  adis_ang_vel[1] = msg.position.x  ;
  adis_ang_vel[2] = msg.position.z ;
 
}

void callbackD435i_raw(const sensor_msgs::Imu::ConstPtr& msg)
{
    
   bmi_ang_vel[0] = msg->angular_velocity.x;
   bmi_ang_vel[1] = msg->angular_velocity.y;
   bmi_ang_vel[2] = msg->angular_velocity.z;
   
   bmi_lin_acc[0] = msg->linear_acceleration.x;
   bmi_lin_acc[1] = msg->linear_acceleration.y;
   bmi_lin_acc[2] = msg->linear_acceleration.z;
   
   
    
    ROS_INFO("r: %.3f, p: %.3f, y: %.3f\n", bmi_ang_vel[0], bmi_ang_vel[1], bmi_ang_vel[2]); 
  

    
  
  
}


void callback_boxPt(const geometry_msgs::Vector3& msg)
{
	
  box[0]= msg.x;
  box[1]= msg.y;
 
}

void callback_vref(const geometry_msgs::Vector3& msg)
{
	
 v_ref[0] = msg.x; //box y dist
 v_ref[1] = msg.y; //linearly scaled vref

}


int main(int argc, char **argv)
{

//	outputFile.open("imuCompare.csv");
//	outputFile << "Roll_3DM" << "," << "Pitch_3DM" << ","  << "Yaw_3DM" << "," << "Roll_16480" << "," <<  "Pitch_16480" << "," <<  "Yaw_16480" << ","  <<"Roll_435i" << "," << "Pitch_435i" << ","  << "Yaw_435i" << std::endl;
	
	outputFile.open("vref.csv");
	outputFile << "boxX" << "," << "boxY" << "," << "y_dist" << "," << "vy_ref" << std::endl;
	

	//raw data
	/*
	outputFile << "x_ang_vel_3DM" << "," << "y_ang_vel_3DM" << ","  << "z_ang_vel_3DM" << "," << "x_lin_acc_3DM" << "," <<  "y_lin_acc_3DM" << "," <<  "z_lin_acc_3DM" << ","  
	<< "x_ang_vel_16480" << "," << "y_ang_vel_16480" << ","  << "z_ang_vel_16480" << "," << "x_lin_acc_16480" << "," <<  "y_lin_acc_16480" << "," <<  "z_lin_acc_16480" << ","
	<< "x_ang_vel_bmi" << "," << "y_ang_vel_bmi" << ","  << "z_ang_vel_bmi" << "," << "x_lin_acc_bmi" << "," <<  "y_lin_acc_bmi" << "," <<  "z_lin_acc_bmi" << ","
	<< std::endl;
*/

    //initialize node 
    ros::init(argc,argv, "exportCSV");

    ros::NodeHandle nh;
   


    ros::Rate loop_rate(100);
    //ros::Subscriber sub = nh.subscribe("/imu/rpy", 10, callback3DM);
	//ros::Subscriber sub2 = nh.subscribe("/mobile_hubo/imuRPY", 10, callback16480);
    //ros::Subscriber sub3 = nh.subscribe("/imu/data", 10, callbackD435i);
    
    
    //ros::Subscriber sub = nh.subscribe("/imu", 10, callback3DM_raw);
    //ros::Subscriber sub3 = nh.subscribe("/camera/imu", 10, callbackD435i_raw);
   // ros::Subscriber sub2 = nh.subscribe("/mobile_hubo/imuvel", 10, callback16480_raw);
   
    ros::Subscriber sub = nh.subscribe("/mobile_hubo/bbox2d", 10, callback_boxPt);
	ros::Subscriber sub2 = nh.subscribe("/mobile_hubo/track", 10, callback_vref);

    
  while (ros::ok())
  {
	
  

		
		//outputFile << roll_a << "," <<  pitch_a << "," <<  yaw_a << "," << roll_b << "," <<  pitch_b << "," <<  yaw_b<< "," << roll_c << "," <<  pitch_c << "," <<  yaw_c <<std::endl;
		//ROS_INFO("r: %.3f, p: %.3f, y: %.3f, r: %.3f, p: %.3f, y: %.3f, r: %.3f, p: %.3f, y: %.3f\n", roll_a, pitch_a, yaw_a, roll_b, pitch_b, yaw_b, roll_c, pitch_c, yaw_c); 
		//ROS_INFO("r: %.3f, p: %.3f, y:  %.3f\n", roll_b, pitch_b, yaw_b); 
		
		//raw data
		/*
		outputFile << gx_ang_vel[0] << "," <<  gx_ang_vel[1] << "," <<  gx_ang_vel[2] << "," << gx_lin_acc[0] << "," <<  gx_lin_acc[1] << "," <<  gx_lin_acc[2]<< "," 
		<< adis_ang_vel[0] << "," <<  adis_ang_vel[1] << "," <<  adis_ang_vel[2] << "," << adis_lin_acc[0] << "," <<  adis_lin_acc[1] << "," <<  adis_lin_acc[2]<< "," 
		<< bmi_ang_vel[0] << "," <<  bmi_ang_vel[1] << "," <<  bmi_ang_vel[2] << "," << bmi_lin_acc[0] << "," <<  bmi_lin_acc[1] << "," <<  bmi_lin_acc[2]
		<< std::endl;
*/

		outputFile << box[0] << "," <<  box[1] << "," <<  v_ref[0] << "," << v_ref[1] << std::endl;



    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

