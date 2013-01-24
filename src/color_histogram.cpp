#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <string>
#include <set>
#include <algorithm>
#include "color_histogram.h"

#define FILENAME "test"

using namespace std;

int main (int argc, char** argv)
{
  
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 
  reader.read (string(argv[1])+".pcd", *cloud);

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);
  PointCloudXYZRGBtoXYZHSV(*cloud,*cloud_hsv);
	
	color_hist object,object2;
	object.init();
  for(size_t i = 0; i < cloud->points.size();++i)
  {
		 object.classify(cloud_hsv->points[i].h,cloud_hsv->points[i].s,cloud_hsv->points[i].v);
  }
	//object2.readFile("est.txt");
	cout << object.count() << endl;
	cout << object << endl;
	//cout << object-object2 << endl;
	object.PrintNorm();
	object.writeFile(string(argv[1])+".color");
	

  return (0);
}
