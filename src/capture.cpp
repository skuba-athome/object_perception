#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <vector>

//
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Vector3.h>

#define WIDTH 1280
#define HIEGHT 1024


IplImage *inFrame  = cvCreateImage(cvSize(WIDTH, HIEGHT), 8, 1);

static int min_x = 0 , min_y = 0 , max_x = WIDTH-1 , max_y = HIEGHT-1 ;

void inFrame_mouse_callback( int event, int x, int y, int flags, void* param ){
	switch( event ){
		case CV_EVENT_LBUTTONDOWN :
			min_x = x;
			min_y =y;
			break;
		case CV_EVENT_RBUTTONDOWN :
			max_x = x;
			max_y = y;
			break;
	}
	//cvRectangle(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int lineType=8, int shift=0)
}

void kinectCallBack(const sensor_msgs::ImageConstPtr& msg)
{
	for(int i=0;i<WIDTH*HIEGHT;i++)
	{
		//printf("%d %d %.2f\n",i/480,i%480,dist[i/480][i%480]);
		/*inFrame->imageData[i*3] = msg->data[i*3+2];
		inFrame->imageData[i*3+1] = msg->data[i*3+1];
		inFrame->imageData[i*3+2] = msg->data[i*3];*/
		inFrame->imageData[i] = msg->data[i];
	}

	cvSmooth(inFrame,inFrame,CV_GAUSSIAN);
	cvRectangle(inFrame,cvPoint(min_x,min_y),cvPoint(max_x,max_y), cvScalarAll(7.0) );
	cvShowImage("inFrame",inFrame);

	char inKey = cvWaitKey(1);
	if(inKey == 27){
		exit(0);
	}
	if(inKey == 32)
	{
		static int count = 0;
		char cstr[20];

		cvSetImageROI( inFrame , cvRect(min_x,min_y,max_x-min_x,max_y-min_y) );

    	sprintf(cstr, "./imgs/tmp/%02d.pgm",count++);

    	cvSaveImage(cstr, inFrame);

    	cvResetImageROI(inFrame);
	}

}
int main(int argc,char * argv[])
{
	ros::init(argc,argv,"capture");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/camera/rgb/image_mono",1,kinectCallBack);


	cvNamedWindow( "inFrame" );

	// Set up the callback
	cvSetMouseCallback( "inFrame", inFrame_mouse_callback );


	ros::spin();
}
