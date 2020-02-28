/* =============================================================
 *
 * This Node was developed for initial testing of "Terrain steeping"
 * of the KI assignment. No TCP. Publish over ROS instead.
 *
 * Vision-related functions were written by Moonyoung Lee.
 *
 * E-mail : ml634@kaist.ac.kr (Moonyoung Lee)

 *
 * Versions :
 * v0.1.0 (20.02.20) 
 * =============================================================
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <string> 

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>

//#include "liftBox/KINECT_DATA.h"
#include "RBLANData.h"

#include "object_detector/bbox3d.h"

//matrix transformation
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//OpenCV for image handling
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>


//to find min,max pixel
#include <algorithm>
#include <math.h>

//ROS Image msg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

//poses 
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"

#define PODO_ADDR "10.12.3.30"
#define PODO_PORT 5500

int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void *);

ros::Publisher      sendKinectData_pub;
ros::Publisher 		marker_publisher; 
//liftBox::KINECT_DATA    TXData;
/* Define the global message to send */
//liftBox::KINECT_DATA msg;

int tcp_size = 0;
int RXDataSize = 0;
int sock = 0;
struct sockaddr_in  server;
pthread_t LANTHREAD_t;
int TXDataSize;
void* TXBuffer;
void* RXBuffer;

int count = 0;
//min,max bound for color threshold
//neon green
int minH = 24, maxH = 46, minS = 120, maxS = 228, minV = 83, maxV = 138;
//int minH = 10, maxH = 27, minS = 32, maxS = 255, minV = 129, maxV = 225;
int boxX = 0;
int boxY = 0;
float boxAngle = 0;
float robotAngle = 0.0;


				
float cameraHeight = 1.55;
float cameraAngle = 35;
float cameraRad = cameraAngle*3.14/180.0;
float tunedOffset = -0.35;


#define opening              2
#define rectangleConst       0
#define contourAreaThreshold 5000
#define boxLikeRatioMax      3.0
#define boxLikeRatioMin      0.3
#define centerPixelX         320
#define centerPixelY         240
#define oneThirdPixelX       210
#define twoThirdPixelX       420
#define twoThirdPixelY       320


LAN_PODO2VISION RXdata;

ros::Publisher pub;

// Global variable for subscribing the data
int numPlanningSteps = 0; 
geometry_msgs::PoseArray stepsArray_pixel;
geometry_msgs::PoseArray stepsArray_pose;

bool pcloud_callback_done = true;

/* initialize CV Window
 * MOONYOUNG 03.21 */
void initVideoWindow() {

    cv::namedWindow("RGBview");
    // cv::namedWindow("HSVview");
    // cv::namedWindow("HSVThreshold");
     cv::namedWindow("HSVFilter");

    cv::startWindowThread();

}

/* display images in Window
 * MOONYOUNG 03.21
   input: Mat of all displayed img */
void displayImages(cv::Mat image1, cv::Mat image2, cv::Mat image3, cv::Mat image4) {


    cv::imshow("RGBview", image1);
    //cv::imshow("HSVview", image2);
    //cv::imshow("HSVThreshold", image3);
    cv::imshow("HSVFilter", image4);

    cv::waitKey(30);

}





void cloud_cb (sensor_msgs::PointCloud2ConstPtr const& input)
{
	
 	

	geometry_msgs::PointStamped base_point;
	//static tf::TransformListener listner;
    //std::cout << "=== handling pointCloud === " << std::endl;
    
    if(numPlanningSteps < 1) return;

    int width = input->width;
    int height = input->height;
    //printf("cloud width: %u, height: %u, rowstep: %u, pointstep: %u\n" , input->width, input->height, input->row_step, input->point_step);

    //get pose for multiple steps positions
    for(int i =0; i < stepsArray_pixel.poses.size() ; i ++)
    {
		
		int arrayPosition = (int)stepsArray_pixel.poses[i].position.y*input->row_step + (int)stepsArray_pixel.poses[i].position.x*input->point_step;
	   //printf("position: %u\n" , arrayPosition);
	   
	   // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + input->fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + input->fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + input->fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &input->data[arrayPosX], sizeof(float));
      memcpy(&Y, &input->data[arrayPosY], sizeof(float));
      memcpy(&Z, &input->data[arrayPosZ], sizeof(float));

		//printf("step : %d, x: %f, y: %f, z: %f\n" , i, X, Y, Z);
		
		//add pose to stepArray
		geometry_msgs::Pose step_pose;
		
		step_pose.position.x = X;
		step_pose.position.y = Y;
		step_pose.position.z = Z;
		
		stepsArray_pose.poses.push_back(step_pose);
		
		
		
	}
	
	
        
        

    
    //geometry_msgs::PointStamped camera_objPose;
    //camera_objPose.header.frame_id = "base_camera";
    //camera_objPose.header.stamp = ros::Time();
    //camera_objPose.point.x = X;
    //camera_objPose.point.y = Y;
    //camera_objPose.point.z = Z;

    ///*transform using TF*/
    
    //try{
    //geometry_msgs::PointStamped base_point;
    //listner.transformPoint("base_link",camera_objPose,base_point);

    ////ROS_INFO("base_camera: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f)",
        //camera_objPose.point.x, camera_objPose.point.y, camera_objPose.point.z,
        //base_point.point.x, base_point.point.y, base_point.point.z);

    ////msg.pos_x = base_point.point.x;
    ////msg.pos_y = base_point.point.y;
   //// msg.pos_z = base_point.point.z;
    

    //count++;
  
    //if (count == 10) 
    //{
		////printf("msg.posx = %f, msg.posy = %f, msg.posz = %f\n",msg.pos_x, msg.pos_y, msg.pos_z );
    
		////write(sock, &msg, TXDataSize);
		
		//count = 0;
	//}
   

  
  //}
  //catch(tf::TransformException& ex){
    //ROS_ERROR("Received an exception trying to transform a point from \"base_camera\" to \"base_link\": %s", ex.what());
  //}



   

}



/* get box X,Y and draw bounding box
 * MOONYOUNG 03.21
   input: all contours of points, Mat to draw on*/
void getBoundingBox(std::vector<std::vector<cv::Point> > contourInput, cv::Mat imageIn)
{
    //print contour size
    //std::cout << "Number of Contour: " << contourInput.size() << std::endl;
    numPlanningSteps = 0;
    
    

    //loop through all detected contour
    for (int contourIndex = 0; contourIndex  < contourInput.size(); contourIndex++)
    {
        //if detected contour is big enough
        //std::cout << "counterarea: " << cv::contourArea(contourInput[contourIndex]) << std::endl;
        if ((cv::contourArea(contourInput[contourIndex])) > contourAreaThreshold)
        {
            //get boundingBox
            //fit an minimum Rect around contour
            cv::RotatedRect box = cv::minAreaRect(contourInput[contourIndex]);
            cv::Point2f box_points[4];
            box.points(box_points);
  
            //to get width, length of rotated box
            float deltaX01, deltaY01, deltaX12, deltaY12;
            deltaX01 = box_points[0].x - box_points[1].x;
            deltaY01 = box_points[0].y - box_points[1].y;
            deltaX12 = box_points[2].x - box_points[1].x;
            deltaY12 = box_points[1].y - box_points[2].y;

            double height = pow( pow(deltaX01,2)+ pow(deltaY01,2) ,0.5);
            double width  = pow( pow(deltaX12,2)+ pow(deltaY12,2) ,0.5);
            
            //std::cout << "height: " << height << std::endl;
            //std::cout << "width: " << width << std::endl;

            //if boundingBox has width/height ratio like box
            if( ((height / width) < boxLikeRatioMax) && ((height / width) > boxLikeRatioMin) )
            {
                //std::cout << " ==== Found Box! ==="  << std::endl;
                
                //add to vector size
                numPlanningSteps = numPlanningSteps + 1;
                
                //draw green bounding box
                //for(int side = 0; side <4; side++) cv::line(imageIn, box_points[side], box_points[(side+1)%4], cv::Scalar(0,255,0),3 );

                //get center position of box
                cv::circle(imageIn,box.center,4,cv::Scalar(0,255,0),2,8,0);
                boxX = box.center.x;
                boxY = box.center.y;
                boxAngle = box.angle;
                //printf("Box X,Y: (%f,%f)\n", box.center.x, box.center.y);
                //std::cout << "Box X, Y: (" << box.center.x << " , " << box.center.y  << ") and width, height: (" << height << ", " << width << ")" << std::endl;
                
                //add to vector
                geometry_msgs::Pose step_pixel;
                step_pixel.position.x = box.center.x;
                step_pixel.position.y = box.center.y;
                stepsArray_pixel.poses.push_back(step_pixel);
                			
				 
            }//end box detected case
        }//end size detected case
    }//end contour loop
    
    //check array of pixel
    for (int i = 0; i < stepsArray_pixel.poses.size(); i++) {
		//ROS_INFO("step: %d, X: %f, Y: %f\n", i, stepsArray_pixel.poses[i].position.x, stepsArray_pixel.poses[i].position.y); 
	}
    
    



}


/*call back function from imageSub
 * MOONYOUNG 03.21
 * input: sensor_msgs
 * output: update global var msg */
void boxImageHandler(const sensor_msgs::ImageConstPtr& msgInput)
{
	//wait until pcloud finished.
	if (!pcloud_callback_done) return;

/* Create tf broadcaster for camera to base link*/
/*
    static tf::TransformBroadcaster broadcaster;
    	broadcaster.sendTransform( tf::StampedTransform( //90+10deg pitch camera (30 but offset)
         tf::Transform(tf::createQuaternionFromRPY(120*3.14/180, 0, 90*3.14/180), tf::Vector3(0.0, 0.0, 1.25)),
         //tf::Transform(tf::Quaternion( -0.5, -0.5, 0.5, 0.5), tf::Vector3(0.0, 0.0, 1.25)),
         ros::Time::now(),"base_link", "base_camera"));
*/

    //convert ROS image to OpenCV img pixel encoding
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msgInput, "bgr8");

    //color change RGB2HSV
    cv::Mat HSVImage;
    cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);

    //threshold HSV range for box
    cv::Mat HSVThreshold;
    cv::inRange(HSVImage, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV),HSVThreshold);

    //opening filter by dilating then eroding image
    cv::Mat HSVFilter;
    cv::Mat morphKernal = cv::getStructuringElement(rectangleConst, cv::Size(9,9), cv::Point(-1,-1) );
    cv::morphologyEx(HSVThreshold,HSVFilter,opening, morphKernal);

    //segment objects by contour
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(HSVFilter, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    //get box info ==> update msg
    getBoundingBox(contours, cv_ptr->image);

    //display
    displayImages(cv_ptr->image, HSVImage, HSVThreshold, HSVFilter);
    
    pcloud_callback_done = false;

}



int main(int argc, char **argv)
{

    std::cout << "\033[1;32m=======================================" << std::endl << std::endl;
    std::cout << "  Node name   : Green Detector" << std::endl << std::endl;
    std::cout << "  version     : 0.1.0" << std::endl;
    std::cout << "  Author      : MoonYoung Lee (ml634@kaist.ac.kr)" << std::endl;

    std::cout << "=======================================\033[0m" << std::endl;


    /* Initialize The Node name */
    ros::init(argc, argv, "liftBox_node");
    /* Define the NodeHandle to communication with ROS system */
	ros::NodeHandle nh;


    /* Loop Cycle = 10Hz = 0.1s */
    ros::Rate loop_rate(10);

    geometry_msgs::PointStamped base_point;
    
    //TF initialize
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster br;
	
	//Create Matrix3x3 from Euler Angles
	tf::Matrix3x3 cam_base_rotation;
	cam_base_rotation.setEulerYPR(0, 0, 0);
	tf::Quaternion quat_cam_base;
	cam_base_rotation.getRotation(quat_cam_base);




    //MOONYOUNG 03.21
    initVideoWindow();

    //ROS image subscribe
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = it.subscribe("/camera/color/image_raw", 1, boxImageHandler);
    
    //tf listener for transform
     tf::TransformListener listener(ros::Duration(10)); //cache time


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
    
    marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("mobile_hubo/step_markers", 1);



    /* spin */
    while(nh.ok())
    {
		
		//enter if processed steps
		if (numPlanningSteps > 0) 
		{
			
			//lookup tranform between cam and base_link frame
			tf::StampedTransform transform;
			try{
			  listener.lookupTransform("/camera_depth_optical_frame", "/base_link",  
									   ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(1.0).sleep();
			}
						
			//std::cout << "inside while loop to publish TF "<< std::endl;
			 
			 //create marker array
			 visualization_msgs::MarkerArray markerArray;
			 
			 //delete old markers
			for(int i = 0; i < stepsArray_pixel.poses.size(); i++){
				visualization_msgs::Marker marker;
				marker.header.stamp = ros::Time::now();
				marker.ns = "foot_step";
				marker.id = i;
				marker.type = visualization_msgs::Marker::CUBE;
				marker.action = visualization_msgs::Marker::DELETEALL;
				marker.lifetime = ros::Duration();

				markerArray.markers.push_back(marker);
			}
			
        
			//publish tf and marker for each step
			for(int i = 0; i < stepsArray_pose.poses.size(); i ++ ) 
			{
				
				//TF broadcasting
				ros::Time now = ros::Time::now();
				tf::Transform tf_cam_steps;

			  
				tf_cam_steps.setOrigin(tf::Vector3(stepsArray_pose.poses[i].position.x,stepsArray_pose.poses[i].position.y, stepsArray_pose.poses[i].position.z));
				tf_cam_steps.setRotation(transform.getRotation());
				
				std::stringstream step_name;
				step_name << "step_" << i;

				br.sendTransform(tf::StampedTransform(tf_cam_steps, ros::Time::now(), "/camera_depth_optical_frame", step_name.str()));
				std::cout << "broadcasted "<< std::endl;
				
				//add marker
				visualization_msgs::Marker marker;
				marker.header.stamp = ros::Time::now();
				marker.header.frame_id = step_name.str();
				marker.ns = "foot_step";
				marker.id = i;
				marker.type = visualization_msgs::Marker::CUBE;
				marker.action = visualization_msgs::Marker::ADD;
				//marker.pose = stepsArray_pose.poses[i];
				marker.scale.x = 0.3;
				marker.scale.y = 0.3;
				marker.scale.z = 0.01;
				marker.color.r = 0.2f;
				marker.color.g = 1.0f;
				marker.color.b = 0.05f;
				marker.color.a = 1.f;
				marker.lifetime = ros::Duration();

				markerArray.markers.push_back(marker);
            
				
			}

        
        //std::cout << "size of marker array: " << markerArray.markers.size() << std::endl;
        
        marker_publisher.publish(markerArray);
        
		
			
        
		}
		
		
		 //remove from vector after publish to prevent infinitely increasing
			while (!stepsArray_pixel.poses.empty())
		  {
			stepsArray_pixel.poses.pop_back();
		  }
		  
		  while (!stepsArray_pose.poses.empty())
		  {
			stepsArray_pose.poses.pop_back();
		  }
		  
		//flag to prevent camera callback repeating
		pcloud_callback_done = true;
		  
		
		

        //MOONYOUNG 03.21
        loop_rate.sleep(); // Go to sleep according to the loop period defined above.
        ros::spinOnce();
 
    }


    return 0;
}

int CreateSocket(const char *addr, int port){
    /* Create Socket with TCP
     * AF_INET = Internet Protocol (TCP/IP)
     * SOCK_STREAM(TCP), SOCK_DGRAM(UDP), SOCK_RAW(RAW) */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1){
        return false;
    }

    server.sin_addr.s_addr = inet_addr(addr);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}

int Connect2Server(){
    /*
     * Connect to PODO server
     */
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (LiftBox)" << std::endl;
    return true;
}

void* LANThread(void *){

    unsigned int tcp_status = 0x00;
    int connectCnt = 0;

    while(1){
        usleep(100);
        if(tcp_status == 0x00){ // If client was not connected
            if(sock == 0){
                CreateSocket(PODO_ADDR, PODO_PORT);
            }
            if(Connect2Server()){
                tcp_status = 0x01;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        
        //MOONYOUNG 05.10 read
        if (tcp_status == 0x01) //connected
        {

            tcp_size = read(sock, RXBuffer, RXDataSize);

            if( 1 )//tcp_size == RXDataSize)
            {
                memcpy(&(RXdata), RXBuffer, RXDataSize);
                RXdata.CoreData.IMU;

            }
        }
    }
    return NULL;
}




