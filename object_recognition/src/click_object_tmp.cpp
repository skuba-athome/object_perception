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
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Vector3.h>
//---pcl---
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


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
//---------
using namespace std;
using namespace cv;

Mat img;
ros::Publisher vector_pub; // = n2.advertise<geometry_msgs::Vector3>("object_point", 1000);
ros::Publisher vector_pub_pointcloud;
IplImage *inFrame  = cvCreateImage(cvSize(640, 480), 8, 3);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);

void depthCb(const sensor_msgs::PointCloud2& cloud) {
  if ((cloud.width * cloud.height) == 0)
    return; //return if the cloud is not dense!
  try {
    pcl::fromROSMsg(cloud, *cloud_pcl);
  } catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Error message: " << e.what());
  }
}



// mouse callback
void on_mouse( int event, int x, int y, int flags, void* param )
{
	if(event == CV_EVENT_LBUTTONUP) 
	{
		ROS_INFO("click_at : x:%d y:%d",x,y);
		geometry_msgs::Vector3 vector;
        cout << "cloud : "<< cloud_pcl->height << ", " << cloud_pcl->width << endl;
		vector.x = cloud_pcl->points[y*640+x].x;
		vector.y = cloud_pcl->points[y*640+x].y;
		vector.z = cloud_pcl->points[y*640+x].z;
		if( vector.x == vector.x 
			&& vector.y == vector.y
			&& vector.z == vector.z
		)
		{
			sensor_msgs::PointCloud2 cloud_tf_out;
			pcl::toROSMsg(*cloud_pcl,cloud_tf_out);
			vector_pub_pointcloud.publish(cloud_tf_out);
			vector_pub.publish(vector);
			printf("send : x:%.2f y:%.2f z:%.2f\n",vector.x,vector.y,vector.z);						
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

/*
void kinectCallBack(const sensor_msgs::ImageConstPtr& msg)
{
        int inKey = 0;

	for(int i=0;i<640*480;i++)
	{
			inFrame->imageData[i*3] = msg->data[i*3];
			inFrame->imageData[i*3+1] = msg->data[i*3+1];
			inFrame->imageData[i*3+2] = msg->data[i*3+2];
	}
		
	cvShowImage("input",inFrame);

	inKey = cvWaitKey(1);
	if(inKey == 27){
		exit(0);
	}
}
*/

int main(int argc , char *argv[])
{
	ros::init(argc,argv,"click_objects");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");


	image_transport::ImageTransport it_(nh);

	image_transport::Subscriber sub_imageColor;
	sub_imageColor = it_.subscribe("/camera/rgb/image_color", 1, kinectCallBack);
	
	//ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,kinectCallBack);
	ros::Subscriber subDepth = n.subscribe("/cloud_tf",1,depthCb);
	//ros::Subscriber subDepth = n.subscribe("/camera/depth_registered/points",1,depthCb);
	
	vector_pub = n.advertise<geometry_msgs::Vector3>("object_point", 1000);
	vector_pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("object_pointcloud", 1000);

	printf("start click_objects\n");
	//cvNamedWindow("input", 1 );
	cvSetMouseCallback("input", on_mouse);
	ros::spin();

}
