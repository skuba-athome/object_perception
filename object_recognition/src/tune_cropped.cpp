/*
 * Description : Do(and read) index using flann & surf
 * Author      : Chanon Onman
 */
#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include <iostream>
#include <vector>

//
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/String.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Vector3.h>
//---pcl---
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl17_ros/transforms.h>
#include <pcl17/point_types.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/vfh.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>

//for webcam resolution 640x480
//#define CENTER_IMAGE_X 313.73619
//#define CENTER_IMAGE_Y 254.26251

#define CENTER_IMAGE_X 614.64112
#define CENTER_IMAGE_Y 352.81378





//---------
using namespace std;
using namespace cv;

std::string logitech_frame = "logitech_cam";

int X_TUNE;
int Y_TUNE;

float FOCAL_LENGTH_X ;
float FOCAL_LENGTH_Y ;
//int FOCAL_LENGTH_X ;
//int FOCAL_LENGTH_Y ;


tf::TransformListener* listener;
Mat img,img_cam;
float currentX=0,currentY=0,currentZ=0;
ros::Publisher vector_pub; // = n2.advertise<geometry_msgs::Vector3>("object_point", 1000);
ros::Publisher vector_pub_pointcloud;
IplImage *inFrame  = cvCreateImage(cvSize(1280, 720), 8, 3);
//IplImage *inFrame  = cvCreateImage(cvSize(640, 480), 8, 3);
static pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud_pcl (new pcl17::PointCloud<pcl17::PointXYZRGB>);

void depthCb(const sensor_msgs::PointCloud2& cloud) {
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_tmp (new pcl17::PointCloud<pcl17::PointXYZ>);
  if ((cloud.width * cloud.height) == 0)
    return; //return if the cloud is not dense!
  try {
      pcl17::fromROSMsg(cloud, *cloud_tmp);

      listener->waitForTransform(logitech_frame, cloud.header.frame_id, cloud.header.stamp, ros::Duration(1.0));
      pcl17_ros::transformPointCloud(logitech_frame, *cloud_tmp, *cloud_pcl, *listener);
  } catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Error message: " << e.what());
  }
}



// mouse callback
void on_mouse( int event, int x, int y, int flags, void* param )
{
	if(event == CV_EVENT_LBUTTONUP) 
	{
		ROS_INFO("click at : x:%d y:%d",x,y);
		float pixel_x = currentX*FOCAL_LENGTH_X/currentZ + CENTER_IMAGE_X;
		float pixel_y = currentY*FOCAL_LENGTH_Y/currentZ + CENTER_IMAGE_Y;
		ROS_INFO("map_to : x:%f y:%f",pixel_x,pixel_y);
		geometry_msgs::Vector3 vector;
        cout << "cloud : "<< cloud_pcl->height << ", " << cloud_pcl->width << endl;
		//vector.x = cloud_pcl->points[y*640+x].x;
		//vector.y = cloud_pcl->points[y*640+x].y;
		//vector.z = cloud_pcl->points[y*640+x].z;
        
        
		vector.x = cloud_pcl->points[y*1280+x].x;
		vector.y = cloud_pcl->points[y*1280+x].y;
		vector.z = cloud_pcl->points[y*1280+x].z;
		if( vector.x == vector.x 
			&& vector.y == vector.y
			&& vector.z == vector.z
		)
		{
            currentX = vector.x;
            currentY = vector.y;
            currentZ = vector.z;

			sensor_msgs::PointCloud2 cloud_tf_out;
			pcl::toROSMsg(*cloud_pcl,cloud_tf_out);
			vector_pub_pointcloud.publish(cloud_tf_out);
			vector_pub.publish(vector);
			printf("send : x:%.3f y:%.3f z:%.3f\n",vector.x,vector.y,vector.z);						
		}

	}
}


void kinectCallBack(const sensor_msgs::ImageConstPtr& msg)
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
		setMouseCallback("My Window", on_mouse, NULL);
		imshow("My Window", img);
		waitKey(3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void webcamCallBack(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img_cam = cv_ptr->image;
		//ROS_INFO("Get image size : %d",img.rows*img.cols);
	
		//show the image
		//while(true){
        //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        
		float pixel_x = currentX*FOCAL_LENGTH_X/currentZ + CENTER_IMAGE_X + X_TUNE;
		float pixel_y = currentY*FOCAL_LENGTH_Y/currentZ + CENTER_IMAGE_Y + Y_TUNE;
        //circle(img_cam, Point(currentX,currentY,currentZ), 10, Scalar(255,255,255),CV_FILLED,8,0)
        circle(img_cam, Point(pixel_x,pixel_y),5, Scalar(255,255,255), 3,0);
		namedWindow("webcam window", 1);
		//setMouseCallback("webcam window", on_mouse, NULL);
		imshow("webcam window", img_cam);
		waitKey(3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

int main(int argc , char *argv[])
{
	ros::init(argc,argv,"click_objects");

	ros::NodeHandle nh("~");
    
    string tmp_focal_x,tmp_focal_y;
    double tmp_x,tmp_y;

    nh.param("X_TUNE", X_TUNE, int(10));
    nh.param("Y_TUNE", Y_TUNE, int(20));

    nh.param("FOCAL_LENGTH_X", tmp_x, 814.033512);
    nh.param("FOCAL_LENGTH_Y", tmp_y, 815.46674);
    //nh.param("FOCAL_LENGTH_X", FOCAL_LENGTH_X, int(814));
    //nh.param("FOCAL_LENGTH_Y", FOCAL_LENGTH_Y, int(815));

    FOCAL_LENGTH_X = (float)tmp_x;
    FOCAL_LENGTH_Y = (float)tmp_y;
    //n.getParam("FOCAL_LENGTH_Y", FOCAL_LENGTH_Y);

    printf("X_TUNE : %d, Y_TUNE : %d, FOCAL_LENGTH_X : %f, FOCAL_LENGTH_Y : %f\n",X_TUNE,Y_TUNE,FOCAL_LENGTH_X,FOCAL_LENGTH_Y);
	image_transport::ImageTransport it_(nh);

	image_transport::Subscriber sub_imageColor,sub_imageColor_cam;
	sub_imageColor = it_.subscribe("/camera/rgb/image_color", 1, kinectCallBack);
	sub_imageColor_cam = it_.subscribe("/logitech_cam/image_raw", 1, webcamCallBack);
	
	//ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,kinectCallBack);
	//ros::Subscriber subDepth = n.subscribe("/cloud_tf",1,depthCb);
	ros::Subscriber subDepth = nh.subscribe("/camera/depth_registered/points",1,depthCb);

	
	vector_pub = nh.advertise<geometry_msgs::Vector3>("object_point", 1000);
	vector_pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("object_pointcloud", 1000);

	printf("start click_objects\n");
	//cvNamedWindow("input", 1 );
	cvSetMouseCallback("input", on_mouse);

	listener = new tf::TransformListener();
	ros::spin();
}
