#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <std_msgs/String.h>
#include <string.h>
#include <cxcore.h>
#include <cvaux.h>
#include <stdlib.h>
#include "surflib.h"
#include "kmeans.h"
#include <ctime>
#include <iostream>

#define WIDTH 1280
#define HIEGHT 1024


using namespace std;
using namespace cv;
using namespace cv_bridge;

// for depthCb()
cv::Mat depthImg ;
cv_bridge::CvImagePtr bridge;
double min_range_;
double max_range_;
float dist[1280][1024];
int canUseDist = 0;


// for capture image
int min_x = 0 ;
int min_y = 0 ;
int max_x = WIDTH-1 ;
int max_y = HIEGHT-1 ;


IplImage *currnetFrame  = cvCreateImage(cvSize(WIDTH, HIEGHT), 8, 1);

