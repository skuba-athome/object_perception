/*********************************************************** 
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/
#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/PointCloud2.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/String.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include<ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "surflib.h"
#include "kmeans.h"
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>

IplImage* imgRGB = cvCreateImage( cvSize(1280,1024),IPL_DEPTH_8U, 3 );
IplImage* img = cvCreateImage( cvSize(1280,1024),IPL_DEPTH_8U, 1 );
int should_continue=1;
ros::Publisher chatter_pub;
std_msgs::String msg_pub;
//-------------------------------------------------------
// In order to you use OpenSURF, the following illustrates
// some of the simple tasks you can do.  It takes only 1
// function call to extract described SURF features!
// Define PROCEDURE as:
//  - 1 and supply image path to run on static image
//  - 2 to capture from a webcam
//  - 3 to match find an object in an image (work in progress)
//  - 4 to display moving features (work in progress)
//  - 5 to show matches between static images
#define PROCEDURE 5

//-------------------------------------------------------

int mainImage(void)
{
  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img=cvLoadImage("imgs/sf.jpg");

  // Detect and describe interest points in the image
  clock_t start = clock();
  surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f); 
  clock_t end = clock();

  std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
  std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;

  // Draw the detected points
  drawIpoints(img, ipts);
  
  // Display the result
  showImage(img);

  return 0;
}

//-------------------------------------------------------

int mainVideo(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Initialise video writer
  //cv::VideoWriter vw("c:\\out.avi", CV_FOURCC('D','I','V','X'),10,cvSize(320,240),1);
  //vw << img;

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img=NULL;

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Extract surf points
    surfDetDes(img, ipts, false, 4, 4, 2, 0.004f);    

    // Draw the detected points
    drawIpoints(img, ipts);

    // Draw the FPS figure
    drawFPS(img);

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------


int mainMatch(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Declare Ipoints and other stuff
  IpPairVec matches;
  IpVec ipts, ref_ipts;
  
  // This is the reference object we wish to find in video frame
  // Replace the line below with IplImage *img = cvLoadImage("imgs/object.jpg"); 
  // where object.jpg is the planar object to be located in the video
  IplImage *img = cvLoadImage("imgs/object.jpg"); 
  if (img == NULL) error("Need to load reference image in order to run matching procedure");
  CvPoint src_corners[4] = {{0,0}, {img->width,0}, {img->width, img->height}, {0, img->height}};
  CvPoint dst_corners[4];

  // Extract reference object Ipoints
  surfDetDes(img, ref_ipts, false, 3, 4, 3, 0.004f);
  drawIpoints(img, ref_ipts);
  showImage(img);

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Main capture loop
  while( true ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);
     
    // Detect and describe interest points in the frame
    surfDetDes(img, ipts, false, 3, 4, 3, 0.004f);

    // Fill match vector
    getMatches(ipts,ref_ipts,matches);
    
    // This call finds where the object corners should be in the frame
    if (translateCorners(matches, src_corners, dst_corners))
    {
      // Draw box around object
      for(int i = 0; i < 4; i++ )
      {
        CvPoint r1 = dst_corners[i%4];
        CvPoint r2 = dst_corners[(i+1)%4];
        cvLine( img, cvPoint(r1.x, r1.y),
          cvPoint(r2.x, r2.y), cvScalar(255,255,255), 3 );
      }

      for (unsigned int i = 0; i < matches.size(); ++i)
        drawIpoint(img, matches[i].first);
    }

    // Draw the FPS figure
    drawFPS(img);

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  // Release the capture device
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------


int mainMotionPoints(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts, old_ipts, motion;
  IpPairVec matches;
  IplImage *img;

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Detect and describe interest points in the image
    old_ipts = ipts;
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

    // Fill match vector
    getMatches(ipts,old_ipts,matches);
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {
      const float & dx = matches[i].first.dx;
      const float & dy = matches[i].first.dy;
      float speed = sqrt(dx*dx+dy*dy);
      if (speed > 5 && speed < 30) 
        drawIpoint(img, matches[i].first, 3);
    }
        
    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  // Release the capture device
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------
void convertmsg2img(const sensor_msgs::ImageConstPtr& msg)
{
	if(msg->width != 1280) return;
	//std::cout<< "kinect: "  << img1->width << std::endl;
	for(int i=0;i<1280*1024;i++)
	{
		imgRGB->imageData[i*3] = msg->data[i*3];
		imgRGB->imageData[i*3+1] = msg->data[i*3+1];
		imgRGB->imageData[i*3+2] = msg->data[i*3+2];
    }
	cvCvtColor ( imgRGB , img , CV_RGB2GRAY );
}

Ipoint findCentroid(IpPairVec matches,int width,int height) {
	Ipoint Centroid;
	Centroid.x = 0;
	Centroid.y = 0;
	for(unsigned int i=0;i<matches.size();++i) {
		if(i==0){
				Centroid.x=matches[i].second.x;
				Centroid.y=matches[i].second.y;
		} else {
				if((abs(matches[i].second.x-Centroid.x)<=width)&&(abs(matches[i].second.y-Centroid.y)<=height)){
					Centroid.x+=matches[i].second.x;
					Centroid.y+=matches[i].second.y;
					Centroid.x/=2;
					Centroid.y/=2;
				}

		}
	}
	return Centroid;
}

struct matchStruct {
	IpPairVec matches;
	Ipoint centroid;
	int matchSize;
};

void find2Candidate(std::vector<struct matchStruct> candidate,struct matchStruct &first,struct matchStruct &second) {
	first = candidate[0];
	for(unsigned int i=1;i<candidate.size();++i) {
		if(candidate[i].matchSize >= first.matchSize) {
			second = first;
			first = candidate[i];
		} else if (candidate[i].matchSize >= second.matchSize) {
			second = candidate[i];
		}
	}
}

void mainStaticMatch(const std_msgs::String::ConstPtr& msg)
{
	//std::ifstream input("pic2/fantaR.txt");
	std::string path("/home/skuba/skuba_athome/objects/");
	std::ifstream input(std::string(path+msg->data+".txt").c_str());    	
	std::string line;
	
  IplImage *img1, *img2;
 	img2=imgRGB; 
  IpVec ipts1, ipts2;
	// environment image
  surfDetDes(img2,ipts2,false,5,4,2,0.0003f);
	//-----------------------------------
	std::vector<std::string> imagename;
	// get all file image
  while( std::getline( input, line ) ) {
    std::cout << line << std::endl;
		imagename.push_back(line);
	}

	std::vector<struct matchStruct> candidate;
	
	for(unsigned int i=0;i < imagename.size();++i)	{
		img1 = cvLoadImage(imagename[i].c_str());
  	surfDetDes(img1,ipts1,false,5,4,2,0.0003f);

  	IpPairVec matches;
  	getMatches(ipts1,ipts2,matches);
		std::cout << "matches : " << matches.size() << std::endl;
		Ipoint centroid = findCentroid(matches,img1->width,img1->height);
		
		struct matchStruct temp;
		temp.matches = matches;
		temp.centroid = centroid;
		temp.matchSize = matches.size();
		candidate.push_back(temp);
	
	}
	struct matchStruct first,second;
	find2Candidate(candidate,first,second);
	//drawPoint(img2,matchesFinal[i].second);
	for (register int i = 0; i < first.matchSize; ++i)
  {
		drawPoint(img2,first.matches[i].second);
	}
	//char filenameS [255];
	//sprintf(filenameS, "%s.jpg",std::string(msg->data+".txt").c_str());
	//cvSaveImage("test.jpg",img2);
	//cvWaitKey(1000);
	int Can1_x=first.centroid.x/2;
	int Can1_y=first.centroid.y/2;
	int Can2_x=second.centroid.x/2;
	int Can2_y=second.centroid.y/2;
	std::stringstream ss;
  ss << "Candiate1 "<< "x="<< Can1_x << " "<< "y="<< Can1_y <<" "<<"size="<< first.matchSize ;
  ss << "|Candiate2 "<< "x="<< Can2_x << " "<< "y="<< Can2_y <<" "<<"size="<< second.matchSize ;
  cvShowImage("2",img2);
	cvWaitKey(1);
	msg_pub.data = ss.str();
	ROS_INFO("%s", msg_pub.data.c_str());
	chatter_pub.publish(msg_pub);
}

//-------------------------------------------------------

int mainKmeans(void)
{
  IplImage *img = cvLoadImage("imgs/img1.jpg");
  IpVec ipts;
  Kmeans km;
  
  // Get Ipoints
  surfDetDes(img,ipts,true,3,4,2,0.0006f);

  for (int repeat = 0; repeat < 10; ++repeat)
  {

    IplImage *img = cvLoadImage("imgs/img1.jpg");
    km.Run(&ipts, 5, true);
    drawPoints(img, km.clusters);

    for (unsigned int i = 0; i < ipts.size(); ++i)
    {
      cvLine(img, cvPoint(ipts[i].x,ipts[i].y), cvPoint(km.clusters[ipts[i].clusterIndex].x ,km.clusters[ipts[i].clusterIndex].y),cvScalar(255,255,255));
    }

    showImage(img);
  }

  return 0;
}

//-------------------------------------------------------

int main(int argc, char **argv) 
{
  if (PROCEDURE == 1) return mainImage();
  if (PROCEDURE == 2) return mainVideo();
  if (PROCEDURE == 3) return mainMatch();
  if (PROCEDURE == 4) return mainMotionPoints();
  //if (PROCEDURE == 5) return mainStaticMatch();
  if (PROCEDURE == 6) return mainKmeans();

	ros::init(argc,argv,"surf");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,convertmsg2img);
	ros::Subscriber sub_filename = n.subscribe("/object/filename",10,mainStaticMatch);
	chatter_pub = n.advertise<std_msgs::String>("/object/surf", 1000);
	ROS_INFO("Start SURF");
	cvNamedWindow("2", CV_WINDOW_NORMAL );
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
