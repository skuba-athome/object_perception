#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <math.h>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/String.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
using namespace cv;  
IplImage* imgRGB = cvCreateImage( cvSize(1280,1024),IPL_DEPTH_8U, 3 );
IplImage* img4 = cvCreateImage( cvSize(1280,1024),IPL_DEPTH_8U, 1 );
ros::Publisher chatter_pub;
std_msgs::String msg_pub;
IplImage *img2,*img1;
IplImage* img=0; 
//cv::Mat image;
int chk=0;
void convertmsg2img(const sensor_msgs::ImageConstPtr& msg)
{
	if(chk==0){
	printf("aaaaaaaaaaaaaaa");
	for(int i=0;i<1280*1024;i++)
	{
		imgRGB->imageData[i*3] = msg->data[i*3];
		imgRGB->imageData[i*3+1] = msg->data[i*3+1];
		imgRGB->imageData[i*3+2] = msg->data[i*3+2];
    }
	//cvCvtColor ( imgRGB , img , CV_RGB2GRAY );
	img2=imgRGB;
	chk=1;
	}
}
void showimg(const std_msgs::String::ConstPtr& msg)
{
	printf("bbbbbbbbbbbbbbbb	");
	//if(img2->width != 1280){
	//cvNamedWindow("kinect", CV_WINDOW_NORMAL );
	//cvShowImage("kinect", imgRGB);
	float x;
	float y;
	char Name [255];
	CvFont font;
   	 cvInitFont(&font,FONT_HERSHEY_COMPLEX, 1.0, 1.0, 0, 4, CV_AA);
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
  	//if(!img) printf("Could not load image file: %s\n",fileName);
	//cvNamedWindow("Ipl", CV_WINDOW_NORMAL );
	//cvShowImage("Ipl",img);
	sscanf(msg->data.c_str(),"%f %f %s",&x,&y,&Name);
	printf("%f %f %s\n",x,y,Name);
	cv::Point Centroid;
	Centroid.x=x*2;
	Centroid.y=y*2;
	//cvPutText(img, Name, Centroid, &font, cvScalar(0, 0, 255,0));
	//cvLine(img2,cvPoint(Centroid.x,0.0),cvPoint(Centroid.x,1024.0), cvScalar(255,0,0),1);
	//cvLine(img2,cvPoint(0.0,Centroid.y),cvPoint(1280.0,Centroid.y), cvScalar(255,0,0),1);
	cvPutText(img2, Name, Centroid, &font, cvScalar(0, 0, 255,0));
	//cvNamedWindow("2", CV_WINDOW_NORMAL );
  	//cvShowImage("2",img);
	cvNamedWindow("22", CV_WINDOW_AUTOSIZE );
  	cvShowImage("22",img2);
	cvWaitKey(1000);
	cvSaveImage("output.jpg",img2);
	//cvReleaseImage(&img);
	//}
	//return 0;

}
int main(int argc, char **argv) 
{
	img=cvLoadImage("pic/frame0000.jpg");
	ros::init(argc,argv,"show");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,convertmsg2img);
	ros::Subscriber sub_filename = n.subscribe("/object/show",10,showimg);
	ros::spin();
	ros::Rate r(10); // 10 hz
/*
	while (should_continue)
	{
	  ros::spinOnce();
	  r.sleep();
	}
*/
}

