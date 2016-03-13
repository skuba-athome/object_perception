#include <cstdlib>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <ClothesDetector.h>
#include <actionlib/server/simple_action_server.h>
#include <clothing_type_classification/FindClothesAction.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "egbis.h"
#include "set"

#define DEFAULT_CLOUD_TOPIC "/camera/depth_registered/points"

using namespace cv;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ClothesDetectionRunner
{
	private:
		ros::NodeHandle nh;
		PointCloudT::Ptr cloud_obj;
        ros::Subscriber cloub_sub;
		actionlib::SimpleActionServer<clothing_type_classification::FindClothesAction> find_clothes_as_;
        clothing_type_classification::FindClothesActionFeedback find_clothes_feedback_;
        clothing_type_classification::FindClothesActionResult find_clothes_result_;

		//-------Clothes Detector Instance-------
		ClothesDetector clothes_detector;
	
	public:
		ClothesDetectionRunner(std::string node_name):
		nh("~"),
		find_clothes_as_(nh, node_name.c_str(), boost::bind(&ClothesDetectionRunner::executeFindClothesCallback, this, _1), false)
		{
			this->cloub_sub = nh.subscribe(DEFAULT_CLOUD_TOPIC, 1, &ClothesDetectionRunner::cloudCallback, this);
		};

		pcl::PointCloud<PointT>::Ptr PassThroughForNormalPlane(pcl::PointCloud<PointT>::Ptr& cloud){
		  pcl::PassThrough<PointT> pass;
		  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
		  pass.setInputCloud (cloud);
		  pass.setFilterFieldName ("z");
		  pass.setFilterLimits (0, 1.5);
		  pass.filter (*cloud_filtered);
		  return(cloud_filtered);
		}

		void find_max_min(float& max_x,float& max_y,float& max_z,float& min_x,float& min_y,float& min_z
		  ,pcl::PointCloud<PointT>::Ptr& cloud_plane ){
		for (float i = 0; i < cloud_plane->points.size (); ++i){
		    if(max_x < cloud_plane->points[i].x){
		      max_x = cloud_plane->points[i].x;
		    }
		    if(max_y < cloud_plane->points[i].y){
		      max_y = cloud_plane->points[i].y;
		    }

		    if(max_z < cloud_plane->points[i].z){
		      max_z = cloud_plane->points[i].z;
		    }
		    if(min_x > cloud_plane->points[i].x){
		      min_x = cloud_plane->points[i].x;
		    }
		    if(min_y > cloud_plane->points[i].y){
		      min_y = cloud_plane->points[i].y;
		    }
		    if(min_z > cloud_plane->points[i].z){
		      min_z = cloud_plane->points[i].z;
		    }
		  }
		}

		void ChangeRBGtoBlack(pcl::PointCloud<PointT>::Ptr& cloud_plane3){
		  for (int i = 0; i < cloud_plane3->width; i++)
		    for (int j = 0; j < cloud_plane3->height; j++)
		    {
		      if(isnan(cloud_plane3->at(i,j).x) || isnan(cloud_plane3->at(i,j).y) || isnan(cloud_plane3->at(i,j).z) )
		      {
		        //Change Nan's RGB to Black
		        cloud_plane3->at(i,j).r = 0;
		        cloud_plane3->at(i,j).g = 0;
		        cloud_plane3->at(i,j).b = 0;

		      }
		  }
		}

		pcl::PointCloud<PointT>::Ptr PassThroughFilter(float& Max_x,float& Max_y,float& Max_z,float& Min_x,float& Min_y,float& Min_z
		  ,pcl::PointCloud<PointT>::Ptr& cloud_for_segmentation){
		  pcl::PassThrough<PointT> pass;
		  pcl::PointCloud<PointT>::Ptr cloud_plane1 (new pcl::PointCloud<PointT> ());
		  pcl::PointCloud<PointT>::Ptr cloud_plane2 (new pcl::PointCloud<PointT> ());
		  pcl::PointCloud<PointT>::Ptr cloud_plane3 (new pcl::PointCloud<PointT> ());
		  pass.setKeepOrganized(true);
		  pass.setInputCloud (cloud_for_segmentation);
		  pass.setFilterFieldName ("z");
		  pass.setFilterLimits (Min_z, Max_z);
		  pass.filter (*cloud_plane1);
		  pass.setInputCloud (cloud_plane1);
		  pass.setFilterFieldName ("y");
		  pass.setFilterLimits (Min_y, Max_y);
		  pass.filter (*cloud_plane2);
		  pass.setInputCloud (cloud_plane2);
		  pass.setFilterFieldName ("x");
		  pass.setFilterLimits (Min_x, Max_x);
		  pass.filter (*cloud_plane3);
		  return(cloud_plane3);
		}

		void runEgbis(pcl::PointCloud<PointT>::Ptr& cloud_plane3){
		  cv::Mat result;
		  cv::Mat egbisImage;
		  if (cloud_plane3->isOrganized()) {
		    result = cv::Mat(cloud_plane3->height, cloud_plane3->width, CV_8UC3);
		    if (!cloud_plane3->empty()) {
		      for (int h=0; h<result.rows; h++) {
		        for (int w=0; w<result.cols; w++) {
		            pcl::PointXYZRGBA point = cloud_plane3->at(w, h);
		            Eigen::Vector3i rgb = point.getRGBVector3i();
		            result.at<cv::Vec3b>(h,w)[0] = rgb[2];
		            result.at<cv::Vec3b>(h,w)[1] = rgb[1];
		            result.at<cv::Vec3b>(h,w)[2] = rgb[0];
		        }
		      }
		    }
		  }
		  int num_ccs;
		  egbisImage = runEgbisOnMat(result, 2.0, 500, 100, &num_ccs);
		}

		void executeFindClothesCallback(const clothing_type_classification::FindClothesGoalConstPtr &goal)
		{
			//TODO-- To Nuttaphon: Implement Your Extract Plane & EGBIS Code Here!!!
			// All the objects needed
		  pcl::PCDReader reader;
		  pcl::NormalEstimation<PointT, pcl::Normal> ne;
		  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
		  pcl::PCDWriter writer;
		  pcl::ExtractIndices<PointT> extract;
		  pcl::ExtractIndices<pcl::Normal> extract_normals;
		  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

		  // Datasets
		  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
		  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
		  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
		  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

		  float Max_x = 0, Max_y = 0, Max_z = 0;
		  float Min_x = 10, Min_y = 10, Min_z = 10;
		  // Read in the cloud data
		  reader.read ("2shirts_1.pcd", *cloud);
		  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

		  // Build a passthrough filter to remove spurious NaNs
		  cloud_filtered = PassThroughForNormalPlane(cloud);
		  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

		  // Estimate point normals
		  ne.setSearchMethod (tree);
		  ne.setInputCloud (cloud_filtered);
		  ne.setKSearch (50);
		  ne.compute (*cloud_normals);

		  // Create the segmentation object for the planar model and set all the parameters
		  seg.setOptimizeCoefficients (true);
		  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
		  seg.setNormalDistanceWeight (0.1);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setMaxIterations (100);
		  seg.setDistanceThreshold (0.03);
		  seg.setInputCloud (cloud_filtered);
		  seg.setInputNormals (cloud_normals);
		  // Obtain the plane inliers and coefficients
		  seg.segment (*inliers_plane, *coefficients_plane);
		  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

		  // Extract the planar inliers from the input cloud
		  extract.setInputCloud (cloud_filtered);
		  extract.setIndices (inliers_plane);
		  extract.setNegative (false);

		  // Write the planar inliers to disk
		  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
		  extract.filter (*cloud_plane);
		  std::cerr << "Cloud after filtering: " << std::endl;
		  find_max_min(Max_x,Max_y,Max_z,Min_x,Min_y,Min_z,cloud_plane);

		  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
		  writer.write ("2shirts_1_plane.pcd", *cloud_plane, false);
		  printf("%f %f %f\n",Max_x,Max_y,Max_z );
		  printf("%f %f %f\n",Min_x,Min_y,Min_z );
		  pcl::PointCloud<PointT>::Ptr cloud_for_segmentation (new pcl::PointCloud<PointT>);

		  reader.read ("2shirts_1.pcd", *cloud_for_segmentation);
		  std::cerr << "PointCloud has: " << cloud_for_segmentation->points.size () << " data points." << std::endl;
		  pcl::PointCloud<PointT>::Ptr final_cloud_plane (new pcl::PointCloud<PointT> ());
		  
		  final_cloud_plane = PassThroughFilter(Max_x,Max_y,Max_z,Min_x,Min_y,Min_z,cloud_for_segmentation);

		  std::cerr << "PointCloud representing the planar component: " << final_cloud_plane->points.size () << " data points." << std::endl;
		  writer.write ("2shirts_1_segmentation_plane.pcd", *final_cloud_plane, false);

		  std::cout << "Input W x H = " << cloud->width << " x " << cloud->height << std::endl;
		  std::cout << "Segmented Cloud W x H = " << final_cloud_plane->width << " x " << final_cloud_plane->height << std::endl;

		  std::cout << "Input Size = " << cloud->size() << " x " << cloud->height << std::endl;
		  std::cout << "Segmented Cloud Size = " << final_cloud_plane->size() << " x " << final_cloud_plane->height << std::endl;

		  //If NaN change rgb to Black Color;
		  ChangeRBGtoBlack(final_cloud_plane);

		  pcl::io::PointCloudImageExtractorFromRGBField<PointT> rgb_extractor;
		  pcl::PCLImage img;
		  rgb_extractor.extract(*final_cloud_plane,img);

		  runEgbis(final_cloud_plane);
		}

		void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
		{
			pcl::fromROSMsg (*cloud_in, *cloud_obj);
		}

};

int main( int argc, char **argv )
{
    ros::init( argc, argv, "clothes_detection_node");
    ClothesDetectionRunner runner(ros::this_node::getName());
    ros::spin();
    return 0;
}	