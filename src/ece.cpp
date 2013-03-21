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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>

std::vector<pcl::PointIndices> cluster_indices;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer ;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

int current_pointer = 0;
void showCloud(int frame);

void keyboardCB(const pcl::visualization::KeyboardEvent& event,void *viewer_void) {
	if(event.getKeySym() == "a" && event.keyUp()) {
		
		current_pointer = (current_pointer+1)%cluster_indices.size();
		std::cout << current_pointer << endl;
		showCloud(current_pointer);
	}
}

void showCloud(int frame) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<int>::const_iterator pit = cluster_indices[frame].indices.begin (); pit != cluster_indices[frame].indices.end (); ++pit)
    cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
	
	
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer->removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_cluster);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cluster,rgb,"sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud");
	viewer->addCoordinateSystem(1.0);
}


int main (int argc, char** argv) {
  std::cout << "Start ece" << std::endl;
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.0001f, 0.0001f, 0.0001f);
  vg.filter (*cloud_filtered);

  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);*/

  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

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

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    if(cloud_plane->points.size() < 100000) break;

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
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  	  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
  	cloud_cluster->width = cloud_cluster->points.size ();
  	cloud_cluster->height = 1;
  	cloud_cluster->is_dense = true;

  	std::cout << j <<" PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
  	std::stringstream ss;
  	ss << "cloud_cluster_" << j << ".pcd";
  	writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
  	j++;
  }

	boost::shared_ptr<pcl::visualization::PCLVisualizer> temp (new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	viewer = temp;
	viewer->initCameraParameters();
	viewer->setBackgroundColor(0,0,0);
	viewer->registerKeyboardCallback(keyboardCB);
  showCloud(0);
	//viewer.registerKeyboardCallback(keyboardCB);
	while(!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}

  return 0;
}
