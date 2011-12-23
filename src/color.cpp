/*
 * Description : Do(and read) index using flann & surf
 * Author      : Chanon Onman
 */
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

using namespace std;
using namespace cv;
using namespace cv_bridge;

#define TOPIC_CONTROL "/cmd_state"
#define MANUAL_MODE  1
#define min_y 370
#define max_y 460

float dist[480][640];
int canPrintDepth = 0;
double min_range_;
double max_range_;
cv::Mat depthImg ;
cv_bridge::CvImagePtr bridge;
ros::Publisher vector_pub; // = n2.advertise<geometry_msgs::Vector3>("object_point", 1000);
IplImage *inFrame  = cvCreateImage(cvSize(640, 480), 8, 3);
IplImage *inFrameHSV  = cvCreateImage(cvSize(640, 480), 8, 3);
int get_dest = 0;

void convertmsg2img(const sensor_msgs::ImageConstPtr& msg);

float g_x = 0 , g_y =0  , g_z=0 ;
int g_c=0 ;
int cut_y= 240;
int get_coke = 0, get_numtip = 0,get_pg = 0;
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB)
{
	float fR, fG, fB;
	float fH, fS, fV;
	const float FLOAT_TO_BYTE = 255.0f;
	const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

	// Create a blank HSV image
	IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
	if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
		printf("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
		exit(1);
	}

	int h = imageRGB->height;		// Pixel height.
	int w = imageRGB->width;		// Pixel width.
	int rowSizeRGB = imageRGB->widthStep;	// Size of row in bytes, including extra padding.
	char *imRGB = imageRGB->imageData;	// Pointer to the start of the image pixels.
	int rowSizeHSV = imageHSV->widthStep;	// Size of row in bytes, including extra padding.
	char *imHSV = imageHSV->imageData;	// Pointer to the start of the image pixels.
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			// Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
			uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
			int bB = *(uchar*)(pRGB+0);	// Blue component
			int bG = *(uchar*)(pRGB+1);	// Green component
			int bR = *(uchar*)(pRGB+2);	// Red component

			// Convert from 8-bit integers to floats.
			fR = bR * BYTE_TO_FLOAT;
			fG = bG * BYTE_TO_FLOAT;
			fB = bB * BYTE_TO_FLOAT;

			// Convert from RGB to HSV, using float ranges 0.0 to 1.0.
			float fDelta;
			float fMin, fMax;
			int iMax;
			// Get the min and max, but use integer comparisons for slight speedup.
			if (bB < bG) {
				if (bB < bR) {
					fMin = fB;
					if (bR > bG) {
						iMax = bR;
						fMax = fR;
					}
					else {
						iMax = bG;
						fMax = fG;
					}
				}
				else {
					fMin = fR;
					fMax = fG;
					iMax = bG;
				}
			}
			else {
				if (bG < bR) {
					fMin = fG;
					if (bB > bR) {
						fMax = fB;
						iMax = bB;
					}
					else {
						fMax = fR;
						iMax = bR;
					}
				}
				else {
					fMin = fR;
					fMax = fB;
					iMax = bB;
				}
			}
			fDelta = fMax - fMin;
			fV = fMax;				// Value (Brightness).
			if (iMax != 0) {			// Make sure its not pure black.
				fS = fDelta / fMax;		// Saturation.
				float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
				if (iMax == bR) {		// between yellow and magenta.
					fH = (fG - fB) * ANGLE_TO_UNIT;
				}
				else if (iMax == bG) {		// between cyan and yellow.
					fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
				}
				else {				// between magenta and cyan.
					fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
				}
				// Wrap outlier Hues around the circle.
				if (fH < 0.0f)
					fH += 1.0f;
				if (fH >= 1.0f)
					fH -= 1.0f;
			}
			else {
				// color is pure Black.
				fS = 0;
				fH = 0;	// undefined hue
			}

			// Convert from floats to 8-bit integers.
			int bH = (int)(0.5f + fH * 255.0f);
			int bS = (int)(0.5f + fS * 255.0f);
			int bV = (int)(0.5f + fV * 255.0f);

			// Clip the values to make sure it fits within the 8bits.
			if (bH > 255)
				bH = 255;
			if (bH < 0)
				bH = 0;
			if (bS > 255)
				bS = 255;
			if (bS < 0)
				bS = 0;
			if (bV > 255)
				bV = 255;
			if (bV < 0)
				bV = 0;

			// Set the HSV pixel components.
			uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
			*(pHSV+0) = bH;		// H component
			*(pHSV+1) = bS;		// S component
			*(pHSV+2) = bV;		// V component
		}
	}
	return imageHSV;
}


// Create an RGB image from the HSV image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated RGB image.
IplImage* convertImageHSVtoRGB(const IplImage *imageHSV)
{
	float fH, fS, fV;
	float fR, fG, fB;
	const float FLOAT_TO_BYTE = 255.0f;
	const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

	// Create a blank RGB image
	IplImage *imageRGB = cvCreateImage(cvGetSize(imageHSV), 8, 3);
	if (!imageRGB || imageHSV->depth != 8 || imageHSV->nChannels != 3) {
		printf("ERROR in convertImageHSVtoRGB()! Bad input image.\n");
		exit(1);
	}

	int h = imageHSV->height;			// Pixel height.
	int w = imageHSV->width;			// Pixel width.
	int rowSizeHSV = imageHSV->widthStep;		// Size of row in bytes, including extra padding.
	char *imHSV = imageHSV->imageData;		// Pointer to the start of the image pixels.
	int rowSizeRGB = imageRGB->widthStep;		// Size of row in bytes, including extra padding.
	char *imRGB = imageRGB->imageData;		// Pointer to the start of the image pixels.
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			// Get the HSV pixel components
			uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
			int bH = *(uchar*)(pHSV+0);	// H component
			int bS = *(uchar*)(pHSV+1);	// S component
			int bV = *(uchar*)(pHSV+2);	// V component

			// Convert from 8-bit integers to floats
			fH = (float)bH * BYTE_TO_FLOAT;
			fS = (float)bS * BYTE_TO_FLOAT;
			fV = (float)bV * BYTE_TO_FLOAT;

			// Convert from HSV to RGB, using float ranges 0.0 to 1.0
			int iI;
			float fI, fF, p, q, t;

			if( bS == 0 ) {
				// achromatic (grey)
				fR = fG = fB = fV;
			}
			else {
				// If Hue == 1.0, then wrap it around the circle to 0.0
				if (fH >= 1.0f)
					fH = 0.0f;

				fH *= 6.0;			// sector 0 to 5
				fI = floor( fH );		// integer part of h (0,1,2,3,4,5 or 6)
				iI = (int) fH;			//		"		"		"		"
				fF = fH - fI;			// factorial part of h (0 to 1)

				p = fV * ( 1.0f - fS );
				q = fV * ( 1.0f - fS * fF );
				t = fV * ( 1.0f - fS * ( 1.0f - fF ) );

				switch( iI ) {
					case 0:
						fR = fV;
						fG = t;
						fB = p;
						break;
					case 1:
						fR = q;
						fG = fV;
						fB = p;
						break;
					case 2:
						fR = p;
						fG = fV;
						fB = t;
						break;
					case 3:
						fR = p;
						fG = q;
						fB = fV;
						break;
					case 4:
						fR = t;
						fG = p;
						fB = fV;
						break;
					default:		// case 5 (or 6):
						fR = fV;
						fG = p;
						fB = q;
						break;
				}
			}

			// Convert from floats to 8-bit integers
			int bR = (int)(fR * FLOAT_TO_BYTE);
			int bG = (int)(fG * FLOAT_TO_BYTE);
			int bB = (int)(fB * FLOAT_TO_BYTE);

			// Clip the values to make sure it fits within the 8bits.
			if (bR > 255)
				bR = 255;
			if (bR < 0)
				bR = 0;
			if (bG > 255)
				bG = 255;
			if (bG < 0)
				bG = 0;
			if (bB > 255)
				bB = 255;
			if (bB < 0)
				bB = 0;

			// Set the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
			uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
			*(pRGB+0) = bB;		// B component
			*(pRGB+1) = bG;		// G component
			*(pRGB+2) = bR;		// R component
		}
	}
	return imageRGB;
}
void controlCallBack(const std_msgs::String::ConstPtr& msg)
{
	if(!strcmp(msg->data.c_str(),"coke"))
	{
		get_coke = 1;
		ROS_INFO("get command : %s\n",msg->data.c_str());
	}
	if(!strcmp(msg->data.c_str(),"numtip"))
	{
		get_numtip = 1;
		ROS_INFO("get command : %s\n",msg->data.c_str());
	}
	if(!strcmp(msg->data.c_str(),"pringles"))
	{
		get_pg = 1;
		ROS_INFO("get command : %s\n",msg->data.c_str());
	}

}

void DepthToWorld(float * x, float * y, float depth)
{
    static const double fx_d = 1.0 / 5.9421434211923247e+02;
    static const double fy_d = 1.0 / 5.9104053696870778e+02;
    static const double cx_d = 3.3930780975300314e+02;
    static const double cy_d = 2.4273913761751615e+02;
    *x = float( (*x - cx_d) * depth * fx_d);
    *y = float( (*y - cy_d) * depth * fy_d);
}


void depthCb( const sensor_msgs::ImageConstPtr& image )
{
    canPrintDepth = 0;
    try
    {
        bridge = cv_bridge::toCvCopy(image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform depth image.");
        return;
    }
   // printf("%d %d \n", bridge->image.cols,bridge->image.rows);
    depthImg = Mat(bridge->image.rows,bridge->image.cols, CV_8UC1);
    for(int i = 0; i < bridge->image.rows; i++)
    {
        float* Di = bridge->image.ptr<float>(i);
        char* Ii = depthImg.ptr<char>(i);
        for(int j = 0; j < bridge->image.cols; j++)
        {
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
            dist[i][j] = Di[j];
        }
    }
    canPrintDepth = 1;
}

// mouse callback
void on_mouse_HSV(int event , int x , int y , int flags , void *param )
{
	if(event == CV_EVENT_LBUTTONUP )
	{
		printf("%d %d %d\n",(unsigned char)inFrameHSV->imageData[(y*640+x)*3],(unsigned char)inFrameHSV->imageData[(y*640+x)*3+1],(unsigned char)inFrameHSV->imageData[(y*640+x)*3+2]);

	}
}
void on_mouse( int event, int x, int y, int flags, void* param )
{
	if(event == CV_EVENT_LBUTTONUP) 
	{
		geometry_msgs::Vector3 vector;
		float tx,ty,tz;
		tx = x;
		ty = y;
		tz = dist[y][x];
		printf("%.2f %.2f %f\n",tx,ty,tz);
		DepthToWorld(&tx,&ty,tz);
		vector.x = tx;
		vector.y = ty;
		vector.z = tz;
		printf("send : x:%.2f y:%.2f z:%.2f\n",vector.x,vector.y,vector.z);						
		vector_pub.publish(vector);
	}
}
int	notBlackorWhite(int i)
{
	if( (unsigned char)inFrame->imageData[i*3] == 0 
		&& (unsigned char)inFrame->imageData[i*3+1] == 0 
		&& (unsigned char)inFrame->imageData[i*3+2] == 0 
		) return 0;
	if( (unsigned char)inFrame->imageData[i*3] == 255 
		&& (unsigned char)inFrame->imageData[i*3+1] == 255 
		&& (unsigned char)inFrame->imageData[i*3+2] == 255 
		) return 0;
	if( (unsigned char)inFrameHSV->imageData[i*3+1] < 30 ) return 0;
	//if( (unsigned char)inFrameHSV->imageData[i*3+2] < 20 ) return 0;
	return 1;
}
int isRed(const IplImage *img,int i)
{
	if( ( (unsigned char)img->imageData[i*3] < 20 
		|| (unsigned char)img->imageData[i*3] > 240 )
		&& notBlackorWhite(i)
	)
		return 1;
	return 0;
}
int isGreen(const IplImage *img,int i)
{
	if( (unsigned char)img->imageData[i*3] >= 30
		&& (unsigned char)img->imageData[i*3] <= 85 
		&& notBlackorWhite(i)
	)
		return 1;
	return 0;
}
int isBlue(const IplImage *img,int i)
{
	if( (unsigned char)img->imageData[i*3] >= 85
		&& (unsigned char)img->imageData[i*3] <= 200
		&& notBlackorWhite(i)
	)
		return 1;
	return 0;
}

/*
	color : 1 = coke(red) , 2 = prinles(green) , 3 = numtip(blue)
*/
void findObject(int color)  
{
	int min_x , max_x ;
	int zone;
	int count_1=0,count_2=0,count_3=0;
	for(int i = min_y * 640 ; i < max_y * 640 ; i++)
	{
		switch(color)
		{
			case 1 : {
					if(isRed(inFrameHSV,i))
					{
						inFrame->imageData[i*3] = 255;	
						inFrame->imageData[i*3+1] = 255;
						inFrame->imageData[i*3+2] = 255;
						if(i%640 < 210)
							count_1++;
						else if(i%640 < 420)
							count_2++;
						else
							count_3++;

					}
					break;
				}
			case 2 : {
					if(isGreen(inFrameHSV,i))
					{
						inFrame->imageData[i*3] = 255;	
						inFrame->imageData[i*3+1] = 255;
						inFrame->imageData[i*3+2] = 255;
						if(i%640 < 210)
							count_1++;
						else if(i%640 < 420)
							count_2++;
						else
							count_3++;

					}
					break;
				}
			case 3 : {
					if(isBlue(inFrameHSV,i))
					{
						inFrame->imageData[i*3] = 255;	
						inFrame->imageData[i*3+1] = 255;
						inFrame->imageData[i*3+2] = 255;
						if(i%640 < 210)
							count_1++;
						else if(i%640 < 420)
							count_2++;
						else
							count_3++;

					}
					break;
				}

			default : printf("ERROR : no object %d \n",color); exit(0); break;
		}

	}/*
	if(count == 0 )  { printf("ERROR : count = 0\n") ;exit(0) ; } // can't find any color in image
	cvLine(inFrame,cvPoint(avg_x,0),cvPoint(avg_x,479),CV_RGB(255,255,255));
	if(avg_x < 210 ) zone = 1;
	else if (avg_x < 420) zone = 2;
	else zone = 3; */
	if(count_1 > count_2 && count_1 > count_3)
		zone=1;
	else if(count_2 > count_3 && count_2 > count_1)
		zone=2;
	else
		zone=3;
	switch(zone)
	{
		case 1 : min_x = 0 ; max_x = 210 ;  break;
		case 2 : min_x = 210 ; max_x = 420; break;
		case 3 : min_x = 420 ; max_x = 640; break;
		default : break;
	}
	float min_dist = 9.9f;
	int ix,iy;
	for(int ty = min_y ; ty < max_y ; ty ++ )
	{
		for(int tx = min_x ; tx < max_x ; tx++)
		{
			if(dist[ty][tx] < min_dist)
			{
				ix = tx;
				iy = ty;
				min_dist = dist[ty][tx];
			}
		}
	}
	if(min_dist == 9.9f) {  printf("ERROR : distance\n"); exit(0) ; }
	float fx = ix;
	float fy = iy;
	float fz = min_dist;

	DepthToWorld(&fx,&fy,fz);
	printf("zone : %d\n",zone);
	printf("%d %d | %.2f %.2f | %.2f\n",ix,iy,fx,fy,min_dist);
	cvCircle(inFrame,cvPoint(ix,iy),5,CV_RGB(255,255,255));

	g_c++;
	g_x+=fx;
	g_y+=fy;
	g_z+=fz;
	if(g_c == 10 )
	{
		geometry_msgs::Vector3 vector;	
		vector.x = g_x/g_c;
		vector.y = g_y/g_c;
		vector.z = g_z/g_c;
		printf("send : x:%.2f y:%.2f z:%.2f\n",vector.x,vector.y,vector.z);						
		vector_pub.publish(vector);
		g_c = 0;
		g_x = 0;
		g_z = 0;
		g_y = 0;
		get_coke = 0;
		get_pg = 0;
		get_numtip = 0;
	}
}
void kinectCallBack(const sensor_msgs::ImageConstPtr& msg)
{
	int inKey = 0;
	bool editLib = false;
	
//if(canPrintDepth) cv::imshow("win2",depthImg);
	//IndexBook *indexBook = load_index(imgLibDir);

	for(int i=0;i<640*480;i++)
	{
		
		if(dist[i/640][i%640] < 1.25)
		{
			//printf("%d %d %.2f\n",i/480,i%480,dist[i/480][i%480]);
			inFrame->imageData[i*3] = msg->data[i*3+2];
			inFrame->imageData[i*3+1] = msg->data[i*3+1];
			inFrame->imageData[i*3+2] = msg->data[i*3];
		}
		else
		{
			inFrame->imageData[i*3] = 0;
			inFrame->imageData[i*3+1] = 0;
			inFrame->imageData[i*3+2] = 0;
		}	
	}		
	//cvShowImage("input",inFrame);
	inFrameHSV = convertImageRGBtoHSV(inFrame);
	//cvShowImage("HSV",inFrameHSV);
	
	cvLine(inFrame,cvPoint(210,0),cvPoint(210,479),CV_RGB(0,0,255));
	cvLine(inFrame,cvPoint(420,0),cvPoint(420,479),CV_RGB(0,0,255));
		
	cvLine(inFrame,cvPoint(0,min_y),cvPoint(639,min_y),CV_RGB(0,0,255));
	cvLine(inFrame,cvPoint(0,max_y),cvPoint(639,max_y),CV_RGB(0,0,255));
	
	//cvShowImage("input",inFrame);
	// red - color
	int chk = 0;
	for(int i=0;i<480*640;i++)
	{
		if( isRed( inFrameHSV , i))
		{
			inFrameHSV->imageData[i*3] = 0;
			chk = 1;
		}
		else if( isGreen(inFrameHSV ,i) )
		{
			inFrameHSV->imageData[i*3] = 45;
			chk = 1;
		}
	}
	inFrame = convertImageHSVtoRGB(inFrameHSV);

	inKey = cvWaitKey(1);
	if(inKey == 27){
		exit(0);
	}
	if(inKey == 65364) // arrow down
	{
		cut_y+=10;
	}
	if(inKey == 65362) // arrow up
	{
		cut_y-=10;
	}
	if(inKey == 'r' && chk)
	{
		findObject(1);
		cvShowImage("out2",inFrame);
	}
	if(inKey == 'g' && chk)
	{
		findObject(2);
		cvShowImage("out2",inFrame);
	}
	if(inKey == 'b' && chk)
	{
		findObject(3);
		cvShowImage("out2",inFrame);
	}
	if(get_coke)
	{
		findObject(1);
	}
	else if(get_pg)
	{
		findObject(2);
	}else if(get_numtip)
	{
		findObject(3);
	}
	/*if(inKey == 'r' && chk)
	{
		int count_x=0;
		int avg_x=0;
		for(int tx=0;tx<640;tx++)
		{
			if(isRed(inFrameHSV,  (cut_y*640)+tx ))
			{
				count_x++;
				avg_x+=tx;
			//	printf("x:%d y:%d\n",avg_x,cut_y);
			}
		}
		avg_x = avg_x/count_x;
		//printf("%d",zone(avg_x))i;
		float xx,yy;
		if( avg_x < 220 )
		{
			int min_x , min_y ;
			float min_dist = 10.0f;
			for(int tx = 0 ; tx < 220 ; tx++ )
			{
				if( dist[cut_y][tx] < min_dist)
				{
					min_x = tx;
					min_y = cut_y;
					min_dist = dist[cut_y][tx];
				}
			}
			xx = min_x;
			yy = min_y;
			DepthToWorld(&xx,&yy,min_dist);
			printf("%d %d | %.2f %.2f | %.2f\n",min_x,min_y,xx,yy,min_dist);
		cvCircle(inFrame,cvPoint(min_x,min_y),5,CV_RGB(255,255,255));
		}
		else if ( avg_x < 440 ) 
		{
			int min_x , min_y ;
			float min_dist = 10.0f;
			for(int tx = 220 ; tx < 440 ; tx++ )
			{
			if( dist[cut_y][tx] < min_dist)
				{
					min_x = tx;
					min_y = cut_y;
					min_dist = dist[cut_y][tx];
				}
			}
			xx = min_x;
			yy = min_y;
			DepthToWorld(&xx,&yy,min_dist);
			int s = (unsigned char)inFrame->imageData[(min_y*640+min_x)*3 + 1];		
			int v = (unsigned char)inFrame->imageData[(min_y*640+min_x)*3 + 2];
			printf("%u %u \n",s,v);
			printf("%d %d | %.2f %.2f | %.2f\n",min_x,min_y,xx,yy,min_dist);
		cvCircle(inFrame,cvPoint(min_x,min_y),5,CV_RGB(255,255,255));
		}
		else
		{
			int min_x , min_y ;
			float min_dist = 10.0f;
			for(int tx = 440 ; tx < 640 ; tx++ )
			{
			if( dist[cut_y][tx] < min_dist)
				{
					min_x = tx;
					min_y = cut_y;
					min_dist = dist[cut_y][tx];
				}
			}
			xx = min_x;
			yy = min_y;
			DepthToWorld(&xx,&yy,min_dist);
			printf("%d %d | %.2f %.2f | %.2f\n",min_x,min_y,xx,yy,min_dist);
		cvCircle(inFrame,cvPoint(min_x,min_y),5,CV_RGB(255,255,255));
		}
		cvShowImage("out2",inFrame);
	}
	if(inKey == 'g' && chk)
	{
		int count_x=0;
		int avg_x=0;
		for(int tx=0;tx<640;tx++)
		{
			if(isGreen(inFrameHSV,  (cut_y*640)+tx ))
			{
				count_x++;
				avg_x+=tx;
			//	printf("x:%d y:%d\n",avg_x,cut_y);
			}
		}
		avg_x = avg_x/count_x;
		//printf("%d",zone(avg_x))i;
		float xx,yy;
		if( avg_x < 220 )
		{
			int min_x , min_y ;
			float min_dist = 10.0f;
			for(int tx = 0 ; tx < 220 ; tx++ )
			{
				if( dist[cut_y][tx] < min_dist)
				{
					min_x = tx;
					min_y = cut_y;
					min_dist = dist[cut_y][tx];
				}
			}
			xx = min_x;
			yy = min_y;
			DepthToWorld(&xx,&yy,min_dist);
			printf("%d %d | %.2f %.2f | %.2f\n",min_x,min_y,xx,yy,min_dist);
		cvCircle(inFrame,cvPoint(min_x,min_y),5,CV_RGB(255,255,255));
		}
		else if ( avg_x < 440 ) 
		{
			int min_x , min_y ;
			float min_dist = 10.0f;
			for(int tx = 220 ; tx < 440 ; tx++ )
			{
			if( dist[cut_y][tx] < min_dist)
				{
					min_x = tx;
					min_y = cut_y;
					min_dist = dist[cut_y][tx];
				}
			}
			xx = min_x;
			yy = min_y;
			DepthToWorld(&xx,&yy,min_dist);
			printf("%d %d \n",inFrame->imageData[(min_y*640+min_x)*3 + 1]),inFrame->imageData[(min_y*640+min_x)*3 +2];
			printf("%d %d | %.2f %.2f | %.2f\n",min_x,min_y,xx,yy,min_dist);
		cvCircle(inFrame,cvPoint(min_x,min_y),5,CV_RGB(255,255,255));
		}
		else
		{
			int min_x , min_y ;
			float min_dist = 10.0f;
			for(int tx = 440 ; tx < 640 ; tx++ )
			{
			if( dist[cut_y][tx] < min_dist)
				{
					min_x = tx;
					min_y = cut_y;
					min_dist = dist[cut_y][tx];
				}
			}
			xx = min_x;
			yy = min_y;
			DepthToWorld(&xx,&yy,min_dist);
			printf("%d %d | %.2f %.2f | %.2f\n",min_x,min_y,xx,yy,min_dist);
		cvCircle(inFrame,cvPoint(min_x,min_y),5,CV_RGB(255,255,255));
		}
		cvShowImage("out2",inFrame);
	}*/

	//cvShowImage("out",inFrame);
/*	if(editLib) {
		write_edited(imgLibDir, indexBook);
	}
*/
	//write_updated(imgLibDir, indexBook);
	cvReleaseImage(&inFrameHSV);
//	delete indexBook;
}

int main(int argc , char *argv[])
{

	float nnRatio   = 0.3f;
	bool reindexing = true;
//	bool camera_running = true;
	CvCapture* capture = 0;
	int frameWidth  = 0;
	int frameHeight = 0;  
/*
	load_index(imgLibDir);
	if(reindexing || !is_updated(imgLibDir)) {
		printf("[Initialize] : reindexing\n");
		do_index(imgLibDir, KDTreeIndex);
	}
*/
	//printf("[Initialize] : reading index\n");
	//write_updated(imgLibDir, indexBook);

	ros::init(argc,argv,"objects");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	
	nh.param("min_range", min_range_, 0.5);
	nh.param("max_range", max_range_, 5.5);

	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,kinectCallBack);
	ros::Subscriber subDepth = n.subscribe("/camera/depth/image",1,depthCb);
	ros::Subscriber sub2 = n.subscribe(TOPIC_CONTROL, 1, controlCallBack);
	vector_pub = n.advertise<geometry_msgs::Vector3>("object_point", 1000);
	

	printf("ros : spin\n");
	//cvNamedWindow("input", 1 );
	//cvNamedWindow("out",1);
	//cvSetMouseCallback("input", on_mouse);
	//cvSetMouseCallback("out",on_mouse_HSV);
	ros::spin();

}
void convertmsg2img(const sensor_msgs::ImageConstPtr& msg)
{
	for(int i=0;i<640*480;i++)
	{
		inFrame->imageData[i*3] = msg->data[i*3+2];
		inFrame->imageData[i*3+1] = msg->data[i*3+1];
		inFrame->imageData[i*3+2] = msg->data[i*3];	
	}				
}
