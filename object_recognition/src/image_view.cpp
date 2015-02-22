#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

Mat img;
vector<int> compression_params; //vector that stores the compression parameters of the image
int frameIndex=20;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	//cout << "in mouce call back method " << endl;
	if  ( event == EVENT_LBUTTONDOWN )
	{
		char fileName[100];
		sprintf(fileName,"frame%04d.png",frameIndex++);
		if(img.empty())
			cout << "Image has no data." << fileName << endl;
		if(imwrite(fileName, img, compression_params)) //write the image to file
			cout << "Save image " << fileName << endl;
		else
			cout << "Failed to save image." << fileName << endl;
	}
}

void imageColorCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img = cv_ptr->image;
		//ROS_INFO("Get image size : %d",img.rows*img.cols);
	
		//show the image
		//while(true){
		namedWindow("My Window", 1);
		setMouseCallback("My Window", CallBackFunc, NULL);
		imshow("My Window", img);
		waitKey(3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"imageViewer");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	image_transport::ImageTransport it_(nh);
	image_transport::Subscriber sub_imageColor;
	sub_imageColor = it_.subscribe("/camera/image_raw", 1, imageColorCb);
	//sub_imageColor = it_.subscribe("/logitech_cam/image_raw", 1, imageColorCb);
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //specify the compression technique
	compression_params.push_back(9);//specify the compression quality

	ros::spin();
	return 0;
}
