
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


ros::Publisher pub;
geometry_msgs::Pose newgoal;




int main(int argc, char **argv)
{

    //initialize node boxDetect
    ros::init(argc,argv, "posepublisher");

    ros::NodeHandle nh;
    

    // Create a ROS publisher for pose
    pub = nh.advertise<geometry_msgs::Pose>("mobile_hubo/goal_pose", 1);

	newgoal.position.x = 3.35;
	newgoal.position.y = 1.2;
	newgoal.orientation.x=0;
	newgoal.orientation.y=0;
	newgoal.orientation.z=0.707;
	newgoal.orientation.w=0.707;
	//90+ degree z 
    
    ros::Rate loop_rate(10);

  while (ros::ok())
  {

 
    pub.publish(newgoal);
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

