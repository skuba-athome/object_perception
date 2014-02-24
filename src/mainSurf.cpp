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
	for(int i=0;i<1280*1024;i++)
	{
		imgRGB->imageData[i*3] = msg->data[i*3];
		imgRGB->imageData[i*3+1] = msg->data[i*3+1];
		imgRGB->imageData[i*3+2] = msg->data[i*3+2];
    }
	cvCvtColor ( imgRGB , img , CV_RGB2GRAY );
}
void mainStaticMatch(const sensor_msgs::ImageConstPtr& msg)
{

	std::ifstream input("est.txt");
    	std::string line;
	
	convertmsg2img(msg);
	//cvNamedWindow("kinect", CV_WINDOW_NORMAL );
	//cvShowImage("kinect", imgRGB);
/*
  IplImage *img1,*img2,*img3,*img4,*img5;
  img1 = cvLoadImage("imgs/01.pgm");
  img2 = cvLoadImage("imgs/02.pgm");
  img3 = cvLoadImage("imgs/03.pgm");
  img4 = cvLoadImage("imgs/04.pgm");
  img5 = cvLoadImage("imgs/05.pgm");

  cvShowImage("1", img1);

  IpVec ipts1, ipts5,ipts4,ipts3,ipts2;
  surfDetDes(img1,ipts1,false,4,4,2,0.0001f);
  surfDetDes(img2,ipts2,false,4,4,2,0.0001f);
  surfDetDes(img3,ipts3,false,4,4,2,0.0001f);
  surfDetDes(img4,ipts4,false,4,4,2,0.0001f);
  surfDetDes(img5,ipts5,false,4,4,2,0.0001f);
 
  IpPairVec matches;

  getMatches(ipts1,ipts2,matches);
 	for (unsigned int i = 0; i < matches.size(); ++i)
  {
    drawPoint(img1,matches[i].first);
    drawPoint(img2,matches[i].second);
  
    const int & w = img1->width;
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
  } 

*/

	/*getMatches(ipts1,ipts3,matches);
	for (unsigned int i = 0; i < matches.size(); ++i)
  {
    drawPoint(img1,matches[i].first);
    drawPoint(img3,matches[i].second);
  
    const int & w = img1->width;
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    cvLine(img3,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
  }
  getMatches(ipts1,ipts4,matches);
  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    drawPoint(img1,matches[i].first);
    drawPoint(img4,matches[i].second);
  
    const int & w = img1->width;
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    cvLine(img4,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
  }
	getMatches(ipts1,ipts5,matches);
	for (unsigned int i = 0; i < matches.size(); ++i)
  {
    drawPoint(img1,matches[i].first);
    drawPoint(img5,matches[i].second);
  
    const int & w = img1->width;
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    cvLine(img5,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
  }
*/
/*
  std::cout<< "Matches: " << matches.size();

  cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
  cvShowImage("2",img2);
  cvShowImage("3", img3);
  cvShowImage("4", img4);
  cvShowImage("5", img5);
  cvShowImage("1", img1);
  cvWaitKey(0);

  return 0;
*/

  IplImage *img1, *img2;
 	img2=imgRGB; 
	int numpic=0;
	IpPairVec matchesFinal;
	Ipoint CentroidFinal;
	int sizematchFinal=0;
	int picFinal=0;
    	while( std::getline( input, line ) ) {
        	std::cout<<line<<'\n';
		//string namepic=line;
		//printf("%s",line);
		char *fileName = (char*)line.c_str();
		img1 = cvLoadImage(fileName);
		//cvNamedWindow(fileName, CV_WINDOW_NORMAL );
  		//cvShowImage(fileName, img1);
		//numpic++;
    	
  //img1 = cvLoadImage("imgs/2013-01-07-030227_1.jpg");
  //img2 = cvLoadImage("imgs/all_1.jpg");
  //img1 = cvLoadImage("pic/est01.jpg");
  //img2 = cvLoadImage("pic/frame0005.jpg");           
  IpVec ipts1, ipts2;
  surfDetDes(img1,ipts1,false,4,4,2,0.0001f);
  surfDetDes(img2,ipts2,false,4,4,2,0.0001f);

  IpPairVec matches;
  getMatches(ipts1,ipts2,matches);
	
  std::cout << matches.size() << std::endl;
  std::cout << ipts1.size() << std::endl;
  std::cout << ipts2.size() << std::endl;
/*
std::cout<< "w1: "  << img1->width << std::endl;
std::cout<< "h1: "  << img1->height << std::endl;
std::cout<< "w2: "  << img2->width << std::endl;
std::cout<< "h2: "  << img2->height << std::endl;
*/
const int & w1 = img1->width;
const int & h1 = img1->height;
//const int & w2 = img2->width;
//const int & h2 = img2->height;


 Ipoint Pt;
 Pt.x=0.0;
 Pt.y=0.0;
drawPoint(img2,Pt);
Ipoint Pt2;
 Pt2.x=1280.0;
 Pt2.y=1024.0;
drawPoint(img2,Pt2);
Ipoint Centroid;
Ipoint CentroidAvg;
int sizematch=0;
int sizematch2=0;
  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    //drawPoint(img1,matches[i].first);
    //drawPoint(img2,matches[i].second);

    const int & w = img1->width;
    //cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    //cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
	printf("x = %f   y=%f\n",matches[i].second.x,matches[i].second.y);

	if(i==0){
	//printf("i = %d",i);

		Centroid.x=matches[i].second.x;
		Centroid.y=matches[i].second.y;
		CentroidAvg.x=matches[i].second.x;
		CentroidAvg.y=matches[i].second.y;
		sizematch2++;
	}

	else{
		//printf("i = %d",i);
		CentroidAvg.x+=matches[i].second.x;
		CentroidAvg.y+=matches[i].second.y;
		if((abs(matches[i].second.x-Centroid.x)<=(w1/2))&&(abs(matches[i].second.y-Centroid.y)<=(h1/2))){
			Centroid.x+=matches[i].second.x;
			Centroid.y+=matches[i].second.y;
			Centroid.x/=2;
			Centroid.y/=2;
			sizematch2++;
		}

	}
	sizematch++;
  }
	CentroidAvg.x=CentroidAvg.x/sizematch;
	CentroidAvg.y=CentroidAvg.y/sizematch;
	//drawPoint(img2,CentroidAvg);

/*
	Centroid.x=Centroid.x/sizematch2;
	Centroid.y=Centroid.y/sizematch2;
*/
	//drawPoint(img2,Centroid);
//printf("CentroidAvg x = %f   y=%f  size=%d\n",CentroidAvg.x,CentroidAvg.y,sizematch);
printf("--------------------------------------------------------------------------\n");
printf("pic : %d\n",numpic);
printf("Centroid x = %f   y=%f  size=%d\n",Centroid.x,Centroid.y,sizematch2);
  //cvLine(img2,cvPoint(Pt.x,Pt.y),cvPoint(Pt2.x,Pt2.y), cvScalar(255,255,255),1);
cvLine(img2,cvPoint(Centroid.x,0.0),cvPoint(Centroid.x,1024.0), cvScalar(255,255,255),1);
cvLine(img2,cvPoint(0.0,Centroid.y),cvPoint(1280.0,Centroid.y), cvScalar(255,255,255),1);
  std::cout<< "Matches: " << matches.size() << std::endl;
printf("--------------------------------------------------------------------------\n");
  //cvNamedWindow("1", CV_WINDOW_NORMAL );
  //cvNamedWindow("2", CV_WINDOW_NORMAL );
  //cvShowImage("1", img1);
  //cvShowImage("2",img2);
  //cvWaitKey(0);
	
    if(numpic==0){
		matchesFinal=matches;
		CentroidFinal=Centroid;
		sizematchFinal=sizematch2;
		picFinal=numpic;
	}
    else{
		if(sizematch2>sizematchFinal){
			matchesFinal=matches;
			CentroidFinal=Centroid;
			sizematchFinal=sizematch2;
			picFinal=numpic;
		}
	}
    numpic++;
}	
printf("--------------------------------------------------------------------------\n");
	printf("picFinal : %d\n",picFinal);
	printf("Centroid x = %f   y=%f  size=%d\n",CentroidFinal.x,CentroidFinal.y,sizematchFinal);
	drawPoint(img2,CentroidFinal);
printf("--------------------------------------------------------------------------\n");
	//drawPoint(img2,matchesFinal[i].second);
	for (unsigned int i = 0; i < matchesFinal.size(); ++i)
  	{
		drawPoint(img2,matchesFinal[i].second);
		
	}
	cvNamedWindow("2", CV_WINDOW_NORMAL );
  	cvShowImage("2",img2);
	cvWaitKey(0);
	should_continue=0;
	
  //return 0;
	
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
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,mainStaticMatch);
	//ros::spin();
	ros::Rate r(10); // 10 hz
	while (should_continue)
	{
	  ros::spinOnce();
	  r.sleep();
	}
}
