#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud_obj (new PointCloudT);

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  pcl::fromROSMsg (*cloud_in, *cloud_obj);
  pcl::io::savePCDFileASCII ("/home/antonio/Documents/test_pcd.pcd", *cloud_obj);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "listener_point_could2");

  ros::NodeHandle n;

  ros::Subscriber cloub_sub = n.subscribe("/depth_registered/depth_registered/points", 1, cloudCallback);
  ros::spin();
  
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  // cloud.width    = 5;
  // cloud.height   = 1;
  // cloud.is_dense = false;
  // cloud.points.resize (cloud.width * cloud.height);
  //
  // for (size_t i = 0; i < cloud.points.size (); ++i)
  // {
  //   cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  // }

  // pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  // std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
  //
  // for (size_t i = 0; i < cloud.points.size (); ++i)
  //   std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
