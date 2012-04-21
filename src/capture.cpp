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
#include "surflib.h"
#include "kmeans.h"
#include <ctime>
#include <iostream>

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

#define TOPIC_CONTROL "/cmd_state"
#define MANUAL_MODE  0

int gROI_x1 = 0;
int gROI_y1 = 0;
int gROI_x2 = 640;
int gROI_y2 = 480;

char *imgLibDir = "./img-lib";

char *queryFile = "query.000.bmp";
char fileName[1024];
int numSaveFrame = 0;
int numObj = 0;
float dist[480][640];
int canPrintDepth = 0;
double min_range_;
double max_range_;
int curObj = 0;
cv::Mat depthImg ;
cv_bridge::CvImagePtr bridge;
ros::Publisher vector_pub; // = n2.advertise<geometry_msgs::Vector3>("object_point", 1000);
typedef struct {
	int numObj;
	char **label;
	int *numPic;
	cv::flann::Index *index;
	cv::Mat desc_mat; // surf descriptor
	cv::Mat ind_mat;  // label(ID)
}IndexBook;
IplImage *inFrame  = cvCreateImage(cvSize(640, 480), 8, 3);

IndexBook *indexBook;
int get_dest = 0;
char obj_label[30];

void convertmsg2img(const sensor_msgs::ImageConstPtr& msg);
IndexBook* load_index(char* dirpath);

float g_x , g_y  , g_z ;
int g_c ;

void controlCallBack(const std_msgs::String::ConstPtr& msg)
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

int mainStaticMatch()
{
  IplImage *img1,*img2,*img3,*img4,*img5;
  img1 = cvLoadImage("imgs/cup01.jpg");
  img2 = cvLoadImage("imgs/cup02.jpg");
  img3 = cvLoadImage("imgs/cup03.jpg");
  img4 = cvLoadImage("imgs/cup04.jpg");
  img5 = cvLoadImage("imgs/cup05.jpg");

	for(int i=0;i<640*480;i++)
	{
		
		if(dist[i/640][i%640] < 1.25 || 1)
		{
			//printf("%d %d %.2f\n",i/480,i%480,dist[i/480][i%480]);
			inFrame->imageData[i*3] = msg->data[i*3+2];
			inFrame->imageData[i*3+1] = msg->data[i*3+1];
			inFrame->imageData[i*3+2] = msg->data[i*3];
		}
		else
		{
			inFrame->imageData[i*3] = 255;
			inFrame->imageData[i*3+1] = 0;
			inFrame->imageData[i*3+2] = 255;
		}	
	}		
	
	//convertmsg2img(msg);
	cvCvtColor(inFrame, grayImg, CV_BGR2GRAY);
	//cvCvtColor(grayImg, markImg, CV_GRAY2RGB);
	if(0){
		findObjectAndMark(grayImg, markImg, inFrame);
	//output
		cvDrawRect(markImg, cvPoint(gROI_x1,gROI_y1), cvPoint(gROI_x2,gROI_y2), CV_RGB(0,255,0));
		cvPutText(markImg, indexBook->label[curObj], cvPoint(gROI_x1,gROI_y1), &cvFont(1.5,2), CV_RGB(255,255,255));

  getMatches(ipts1,ipts2,matches);
 	for (unsigned int i = 0; i < matches.size(); ++i)
  {
    drawPoint(img1,matches[i].first);
    drawPoint(img2,matches[i].second);
  
    const int & w = img1->width;
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
  } 
	getMatches(ipts1,ipts3,matches);
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

	inKey = cvWaitKey(1);
	if(inKey == 27){
		exit(0);
	}
	else if(inKey == 32){
		static int picID = 0;
		printf("fetch\n");
		cvSetImageROI(inFrame, cvRect(gROI_x1, gROI_y1, gROI_x2 - gROI_x1, gROI_y2 - gROI_y1));
		
		sprintf(fileName,"imgs/%03d.bmp",picID++);
		cvSaveImage(fileName, inFrame);
		cvResetImageROI(inFrame);
	}
	else if(inKey == 2555904){
		curObj = (curObj+1)%indexBook->numObj;
	}
	else if(inKey == 2424832){
		curObj = (curObj+(indexBook->numObj-1))%indexBook->numObj;
	}
	else if(inKey == 'a'){
		get_dest = 1;
		strcpy(obj_label,"oishi");
	}
	else if(inKey >= 0){
		printf("key = %d\n", inKey);
	}

  cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
  cvShowImage("2",img2);
  cvShowImage("3", img3);
  cvShowImage("4", img4);
  cvShowImage("5", img5);
  cvShowImage("1", img1);
  cvWaitKey(0);

  return 0;
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


    showImage(img);
  }

  return 0;
}

//-------------------------------------------------------

int main(void) 
{
  if (PROCEDURE == 1) return mainImage();
  if (PROCEDURE == 2) return mainVideo();
  if (PROCEDURE == 3) return mainMatch();
  if (PROCEDURE == 4) return mainMotionPoints();
  if (PROCEDURE == 5) return mainStaticMatch();
  if (PROCEDURE == 6) return mainKmeans();
}
