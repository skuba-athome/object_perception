// ROS core
#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
//
#include <opencv2/highgui/highgui.hpp>

#include "color_histogram.h"

#define PICTURE_TYPE ".bmp"

using namespace std;

static int collection_count;
static int current_pic;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
static vector<pcl::PointIndices> cluster_indices;
static string filename;
static pcl::PointXYZRGB pointCheck;
static ros::Publisher pub_color_hist;

void savePointCloudToPCD(const sensor_msgs::PointCloud2& cloud,const char filename[]) {
     pcl::io::savePCDFile (filename, cloud, Eigen::Vector4f::Zero (),Eigen::Quaternionf::Identity (), false);
     ROS_INFO ("Data saved to %s", filename);
} 

void ece(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,const float &voxel_distance,const string filename) {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
     
     ROS_INFO ("PointCloud before filtering has: %d", cloud->points.size());
     // Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<pcl::PointXYZRGB> vg;
     vg.setInputCloud (cloud);
     vg.setLeafSize (voxel_distance, voxel_distance,voxel_distance);
     vg.filter (*cloud_filtered);
     
     ROS_INFO ("PointCloud after filtering has: %d", cloud_filtered->points.size());
     
     // Create the segmentation object for the planar model and set all the parameters
     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
     pcl::PCDWriter writer;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (100);
     seg.setDistanceThreshold (0.02);
     
     int nr_points = (int) cloud_filtered->points.size ();
     while (cloud_filtered->points.size () > 0.3 * nr_points) {
         // Segment the largest planar component from the remaining cloud
         seg.setInputCloud (cloud_filtered);
         seg.segment (*inliers, *coefficients);
         if (inliers->indices.size () == 0) {
             ROS_INFO ("Could not estimate a planar model for the given dataset.");
             break;
         }
         
         // Extract the planar inliers from the input cloud
         pcl::ExtractIndices<pcl::PointXYZRGB> extract;
         extract.setInputCloud (cloud_filtered);
         extract.setIndices (inliers);
         extract.setNegative (false);
         
         // Write the planar inliers to disk
         extract.filter (*cloud_plane);
         ROS_INFO ("PointCloud representing the planar component: %d data points.",cloud_plane->points.size());
         
         // Remove the planar inliers, extract the rest
         extract.setNegative (true);
         extract.filter (*cloud_f);
         cloud_filtered = cloud_f;
     }
     
     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
     tree->setInputCloud (cloud_filtered);
     
     pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
     ec.setClusterTolerance (0.02); // 2cm
     ec.setMinClusterSize (1000);
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered);
     ec.extract (cluster_indices);

	 int j = 0;
     for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
         for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
             cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		 }
         cloud_cluster->width = cloud_cluster->points.size ();
         cloud_cluster->height = 1;
         cloud_cluster->is_dense = true;
         
         ROS_INFO("PointCloud representing the Cluster: %d data points.",cloud_cluster->points.size ());
         stringstream ss;
         ss << "object_" << j << ".pcd";
		 j++;
         writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
		  
     }
}

void getImageFromPCD(const char filename[]) {
     
     
}

void showCollectData(const int& pictureNO) {
	 stringstream sss;
	 sss << "picture_" << pictureNO << PICTURE_TYPE;
	 cv::Mat image;
	 image = cv::imread(sss.str(),1);
	 cv::imshow("Show Image",image);
     //image.release();
}

void filenameCallback (const std_msgs::String::ConstPtr& msg) {
	filename = msg->data;
	ROS_INFO("filename : %s",msg->data.c_str());
	cluster_indices.clear();
	ece(cloud_pcl,0.00001f,filename);
}

void checkPoint() {
	 color_hist object_main;
	 object_main.readFile(filename+".color");
     int isFound = 0;
	int j=0;
     for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
	 	if(isFound) continue;
		int isContinue = 1;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
		    if(pointCheck.x == cloud_filtered->points[*pit].x &&
		   	pointCheck.y == cloud_filtered->points[*pit].y &&
		   	pointCheck.z == cloud_filtered->points[*pit].z )
		   		isContinue = 0;
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		}
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
		 
		if(isContinue) continue;
		isFound = 1;
	
     	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);
	 	PointCloudXYZRGBtoXYZHSV(*cloud_cluster,*cloud_hsv);
		
		color_hist object_temp;
	    object_temp.init();
	    object_temp.classify(cloud_hsv);
		
		std::stringstream sk;
		//std::cout << object_main << object_temp ;
		sk << j << " " << filename << " " << object_main - object_temp;
		std_msgs::String msg;
		msg.data = sk.str();
		pub_color_hist.publish(msg);
		ROS_INFO("%s",msg.data.c_str());
     }
     if(isFound == 0) {
	 	std::stringstream sk;
	 	//std::cout << object_main << object_temp ;
	 	sk << j << " " << filename << " " << 100.00f;
	 	std_msgs::String msg;
	 	msg.data = sk.str();
	 	pub_color_hist.publish(msg);
	 	ROS_INFO("%s",msg.data.c_str());
     }
}

void pointcheckCallback (const std_msgs::String::ConstPtr& msg) {
	int x,y;
	ROS_INFO("pointCheck : %s",msg->data.c_str());
	sscanf(msg->data.c_str(),"%d,%d",&x,&y);
	if(x+y*640 >= 0 && x+y*640 < 640*480) {
		pointCheck = cloud_pcl->points[x+y*640];
		checkPoint();
	}
}


void saveCloudCB(const sensor_msgs::PointCloud2& cloud) {
	if ((cloud.width * cloud.height) == 0)
	  return; //return if the cloud is not dense!
	try {
      	std::stringstream ss;
      	ss << cloud.header.stamp << ".pcd";

		//savePointCloudToPCD(cloud,ss.str().c_str());
		pcl::fromROSMsg(cloud, *cloud_pcl);
	} catch (std::runtime_error e) {
		ROS_ERROR_STREAM("Error message: " << e.what());
	}
}

void mouseCallBack(int event, int x, int y, int flags,void* param){
	switch( event )	{
		case CV_EVENT_LBUTTONDOWN:
			current_pic = (current_pic + 1)%collection_count;
			showCollectData(current_pic);
	}
}

int main (int argc, char** argv) {
	ros::init(argc,argv,"object_color_hist");

	ros::NodeHandle n;
	ros::Subscriber sub_pcl = n.subscribe("/camera/depth_registered/points",1,saveCloudCB);
	ros::Subscriber sub_filename = n.subscribe("/object/filename",1,filenameCallback);
	ros::Subscriber sub_pointcheck = n.subscribe("/object/pointcheck",1,pointcheckCallback);
	pub_color_hist = n.advertise<std_msgs::String>("/object/color_hist",1000);
	
    ROS_INFO("Start color_hist");
	//cv::namedWindow("Show Image",CV_WINDOW_AUTOSIZE);
	//cvSetMouseCallback("Show Image",mouseCallBack);
	ros::Rate r(10);
	ros::spin();
	return 0;
}
