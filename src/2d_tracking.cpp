/* =============================================================
 *
 * This Node was developed to perform "Box Lift mission"
 * of the KI assignment.
 *
 * Vision-related functions were written by Moonyoung Lee.
 * Communication-related functions were written by Yujin Heo.
 *
 * E-mail : ml634@kaist.ac.kr (Moonyoung Lee)
 * E-mail : blike4@kaist.ac.kr (Yujin Heo)
 *
 * Versions :
 * v0.1.0 (18.03.20) Connect PODO by TCP/IP
 * =============================================================
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "object_detector/bbox2d.h"


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




int count = 0;
//min,max bound for color threshold
int minH = 11, maxH = 26, minS = 120, maxS = 255, minV = 126, maxV = 212;
//int minH = 10, maxH = 27, minS = 32, maxS = 255, minV = 129, maxV = 225;
int boxX = 0;
int boxY = 0;
float boxAngle = 0;


				
float cameraHeight = 1.55;
float cameraAngle = 35;
float cameraRad = cameraAngle*3.14/180.0;
float tunedOffset = -0.35;


#define opening              2
#define rectangleConst       0
#define contourAreaThreshold 100
#define boxLikeRatio         3.5
#define centerPixelX         320
#define centerPixelY         240
#define oneThirdPixelX       210
#define twoThirdPixelX       420
#define twoThirdPixelY       320



geometry_msgs::Vector3 box_point_msg;
geometry_msgs::Vector3 track_vel_msg;

ros::Publisher box_msg_pub;
ros::Publisher track_msg_pub;

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




/* get box X,Y and draw bounding box
 * MOONYOUNG 03.21
   input: all contours of points, Mat to draw on*/
void getBoundingBox(std::vector<std::vector<cv::Point> > contourInput, cv::Mat imageIn)
{
    //print contour size
    //std::cout << "Number of Contour: " << contourInput.size() << std::endl;

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

            int height = pow( pow(deltaX01,2)+ pow(deltaY01,2) ,0.5);
            int width  = pow( pow(deltaX12,2)+ pow(deltaY12,2) ,0.5);
            
            //std::cout << "height: " << height << std::endl;
            //std::cout << "width: " << width << std::endl;

            //if boundingBox has width/height ratio like box
            if( (height / width) < boxLikeRatio)
            {
                //std::cout << " ==== Found Box! ==="  << std::endl;
                //draw green bounding box
                for(int side = 0; side <4; side++) cv::line(imageIn, box_points[side], box_points[(side+1)%4], cv::Scalar(0,255,0),3 );

                //get center position of box
                cv::circle(imageIn,box.center,4,cv::Scalar(0,255,0),2,8,0);
                boxX = box.center.x;
                boxY = box.center.y;
                boxAngle = box.angle;
                //printf("Box X,Y: (%f,%f)\n", box.center.x, box.center.y);
//                std::cout << "Box X, Y: (" << box.center.x << " , " << box.center.y  << ") and width, height: (" << height << ", " << width << ")" << std::endl;


				
				//darw line from robot origin to box center
				cv::Point2f robotOrigin(320, 480);
				cv::Point2f robotOriginText(325, 470);
				//cv::line(imageIn, robotOrigin, box.center, cv::Scalar(255,0,0),3);

                box_point_msg.x = boxX;
                box_point_msg.y = boxY;
                box_msg_pub.publish(box_point_msg);



				 
            }//end box detected case
        }//end size detected case
    }//end contour loop





}

//linearly scale y to Vy 
//publish vy
void tracking(float y, float depth)
{

    float Vy_ref = 0.0; //to publish

    int direction = 1; //+1 if +y (left) -1 if =y (right) 

    if(y < 0) 
    {
        direction = -1;
    }

    float Vymax = 0.25 * direction;
    float Vymin = 0.0;

    float obj_max_y = 1.0 * direction;
    float obj_min_y = 0.05 * direction;

    
    if(depth > 0.3)
    {

        Vy_ref = ((Vymax - Vymin) * (y - obj_min_y) ) / (obj_max_y -obj_min_y);
        //printf("y: %f, v_ref: %f\n" , y, Vy_ref);

        track_vel_msg.x = y;
        track_vel_msg.y = Vy_ref;
        track_msg_pub.publish(track_vel_msg);


    }

    else
    {
        Vy_ref = 0;
        track_vel_msg.x = y;
        track_vel_msg.y = Vy_ref;
        track_msg_pub.publish(track_vel_msg);

    }


   



}


/*call back function from imageSub
 * MOONYOUNG 03.21
 * input: sensor_msgs
 * output: update global var msg */
void boxImageHandler(const sensor_msgs::ImageConstPtr& msgInput)
{

/* Create tf broadcaster for camera to base link*/
    static tf::TransformBroadcaster broadcaster;
    	broadcaster.sendTransform( tf::StampedTransform( /*90+10deg pitch camera (30 but offset)*/
         tf::Transform(tf::createQuaternionFromRPY(120*3.14/180, 0, 90*3.14/180), tf::Vector3(0.0, 0.0, 1.25)),
         //tf::Transform(tf::Quaternion( -0.5, -0.5, 0.5, 0.5), tf::Vector3(0.0, 0.0, 1.25)),
         ros::Time::now(),"base_link", "base_camera"));


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

}


void cloud_cb (sensor_msgs::PointCloud2ConstPtr const& input)
{
    
    
    geometry_msgs::PointStamped base_point;
    static tf::TransformListener listner;
    //std::cout << "=== handling pointCloud === " << std::endl;

    int width = input->width;
    int height = input->height;
    // printf("cloud width: %u, height: %u, rowstep: %u, pointstep: %u\n" , input->width, input->height, input->row_step, input->point_step);
    int arrayPosition = boxY*input->row_step + boxX*input->point_step;
    // printf("position: %u\n" , arrayPosition);

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

    //printf("x: %f, y: %f, z: %f\n" , X, Y, Z);

      tracking(-X, Z);



}






int main(int argc, char **argv)
{

    std::cout << "\033[1;32m=======================================" << std::endl << std::endl;
    std::cout << "  Node name   : LiftBox" << std::endl << std::endl;
    std::cout << "  version     : 0.1.0" << std::endl;
    std::cout << "  Author      : MoonYoung Lee (ml634@kaist.ac.kr)" << std::endl;
    std::cout << "  Author      : Yujin Heo (blike4@kaist.ac.kr)" << std::endl << std::endl;

    std::cout << "  Step    0   : Connect Kinnect USB " << std::endl << std::endl;
    std::cout << "  Run CMD 1   : $ roscore" << std::endl << std::endl;
    std::cout << "  Run CMD 2   : $ roslaunch freenect_launch freenect.launch" << std::endl << std::endl;
    std::cout << "  Run CMD 3   : $ rosrun liftBox liftBox" << std::endl << std::endl;


    std::cout << "=======================================\033[0m" << std::endl;


    /* Initialize The Node name */
    ros::init(argc, argv, "liftBox_node");
    /* Define the NodeHandle to communication with ROS system */
	ros::NodeHandle nh;




    


    /* Loop Cycle = 10Hz = 0.1s */
    ros::Rate loop_rate(10);

    
    
    
    //MOONYOUNG 03.21
    initVideoWindow();

    //ROS image subscribe
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = it.subscribe("/camera2/color/image_raw", 1, boxImageHandler);

    box_msg_pub = nh.advertise<geometry_msgs::Vector3>("/mobile_hubo/bbox2d", 10);
    track_msg_pub = nh.advertise<geometry_msgs::Vector3>("/mobile_hubo/track", 10);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera2/depth_registered/points", 1, cloud_cb);

    ros::spin();

    /* write pos and orientation data with TCP */
    while(nh.ok())
    {


        //MOONYOUNG 03.21
        loop_rate.sleep(); // Go to sleep according to the loop period defined above.
        //handle ros communication events
    }


    return 0;
}


