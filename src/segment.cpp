#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include "rosbag/bag.h"
#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "rosbag/view.h"
#include "rosbag/query.h"
#include "rosbag/message_instance.h"
#include <boost/foreach.hpp>
#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>


typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud_t;

void callBackFn(const sensor_msgs::ImageConstPtr& imgColorPtr){
	sensor_msgs::CvBridge bridge;
	IplImage* imgColor = bridge.imgMsgToCv(imgColorPtr, "bgr8");
	cvNamedWindow("image",1); cvShowImage("image", imgColor);
	cvWaitKey(100);
	//cvReleaseImage(&imgColor);
}

int imgCount = 0;
void callBackReadPtCloud(const Cloud_t::ConstPtr pts2){
 	if (pts2 != NULL){
		Cloud_t cloud = *pts2;
		int width  = cloud.width;
		int height = cloud.height;
		IplImage* img =cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		ofstream file3D;
		char fileNamePts[100];
		sprintf(fileNamePts,"3dPoints_%d.xyz",imgCount);
    		file3D.open(fileNamePts, ios::out);
	  	for(int i=0; i < height; i++){
			unsigned char* imgPtr = (unsigned char*)(img->imageData + i*img->widthStep);
			for(int j=0; j < width; j++){
				unsigned char* rgb_ptr 	     = (unsigned char*)&(cloud.points[i*width+j].rgb);
				*(imgPtr+j*3) =  (*(rgb_ptr+0));
				*(imgPtr+j*3+1) =  (*(rgb_ptr+1));
				*(imgPtr+j*3+2) =  (*(rgb_ptr+2));
				//write the xyz values into a file!
				file3D << cloud.points[i*width+j].x <<
					"  " << cloud.points[i*width+j].y <<
					" " << cloud.points[i*width+j].z <<"\n";
			}
		}
		file3D.close();
		char fileNameImg[100];
		sprintf(fileNameImg,"cloud_image_%d.png",imgCount);
		cvNamedWindow("image",1); cvShowImage("image",img);
		cvWaitKey(-1);
		cvSaveImage(fileNameImg,img);
		imgCount++;
		cvReleaseImage(&img);
		exit(1);
  	}
}


int main(int argc, char* argv[]){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1, callBackReadPtCloud);
	ros::spin();
	return 0;
}
