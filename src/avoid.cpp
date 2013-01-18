#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


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

ros::Publisher pub;
ros::Publisher cloudpub_;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	sensor_msgs::PointCloud2 cloud_out;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::fromROSMsg(*input , *cloud);

	std::cout << cloud->height << " " << cloud->width << std::endl;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.4, 2.0);
	pass.filter (*cloud);


	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*


	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.02f, 0.02f, 0.02f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


	pcl::toROSMsg(*cloud_filtered , cloud_out);

	cloud_out.header = input->header;
	cloudpub_.publish(cloud_out);
		
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("input", 1);

	cloudpub_ = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered",1);
	// Spin
	ros::spin ();
}
