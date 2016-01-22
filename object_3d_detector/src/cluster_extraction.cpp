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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
int
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2 (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.8);
  pass.filter (*cut_cloud);
  writer.write<pcl::PointXYZ> ("cut_cloud.pcd", *cut_cloud, false);

  // Estimate point normals
  ne.setSearchMethod (tree_2);
  ne.setInputCloud (cut_cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);


  seg.setNormalDistanceWeight (0.1);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setProbability (0.99);
  seg.setInputCloud (cut_cloud);
  seg.setInputNormals (cloud_normals);

  seg.segment (*inliers_plane, *coefficients_plane);

  extract.setInputCloud (cut_cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter (*cloud_plane);
  std::cout << "Size:  " << cloud_plane->points.size() << std::endl;
  float x_min_plane=9999;
  float y_min_plane=9999;
  float z_min_plane=9999;
  float x_max_plane=-1;
  float y_max_plane=-1;
  float z_max_plane=-1;

  float down_y_plane=9999;
  float up_y_plane=-9999;
  float right_x_plane=-9999;
  float left_x_plane=9999;
  float pos_z_plane=-9999;
  float neg_z_plane=9999;

  for(int i=0; i<cloud_plane->points.size(); i++){
    float x=std::abs(cloud_plane->points[i].x);
    float y=std::abs(cloud_plane->points[i].y);
    float z=std::abs(cloud_plane->points[i].z);

    x_min_plane = std::min(x_min_plane,x);
    y_min_plane = std::min(y_min_plane,y);
    z_min_plane = std::min(z_min_plane,z);

    x_max_plane = std::max(x_max_plane,x);
    y_max_plane = std::max(y_max_plane,y);
    z_max_plane = std::max(z_max_plane,z);

    left_x_plane = std::min(left_x_plane,cloud_plane->points[i].x);
    down_y_plane = std::min(down_y_plane,cloud_plane->points[i].y);
    neg_z_plane = std::min(neg_z_plane,cloud_plane->points[i].z);

    right_x_plane = std::max(right_x_plane,cloud_plane->points[i].x);
    up_y_plane = std::max(up_y_plane,cloud_plane->points[i].y);
    pos_z_plane = std::max(pos_z_plane,cloud_plane->points[i].z);
  }
  // if (x_max_plane > 1){
  //     x_max_plane = 1;
  // }
  // if (y_max_plane > 1){
  //   y_max_plane = 1;
  // }
  // if (pos_z_plane > 0.6){
  //   pos_z_plane = 0.6;
  // }

  std::cout << "width_plane  " << std::abs(left_x_plane-right_x_plane) << std::endl;
  std::cout << "height_plane   " << std::abs(down_y_plane-up_y_plane) << std::endl;
  std::cout << "depth_plane   " << std::abs(neg_z_plane-pos_z_plane) << std::endl;
  std::cout << "down_y_plane  " << down_y_plane << std::endl;
  std::cout << "up_y_plane  " << up_y_plane << std::endl;
  std::cout << "right_x_plane  " << right_x_plane << std::endl;
  std::cout << "left_x_plane  " << left_x_plane << std::endl;
  std::cout << "pos_z_plane  " << pos_z_plane << std::endl;
  std::cout << "neg_z_plane  " << neg_z_plane << std::endl;


  // pcl::VoxelGrid<pcl::PointXYZ> vg2;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_plan (new pcl::PointCloud<pcl::PointXYZ>);
  // vg2.setInputCloud (cloud_plane);
  // vg2.setLeafSize (0.01f, 0.01f, 0.01f);
  // vg2.filter (*cloud_filtered_plan);
  // std::cout << "PointCloud after filtering has: " << cloud_filtered_plan->points.size ()  << " data points." << std::endl; //*
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_plan (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud_plane);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0, 0.4);
  // pass.filter (*cloud_filtered_plan);

  writer.write<pcl::PointXYZ> ("plan_filtter.pcd", *cloud_plane, false);

  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // int i=0, nr_points = (int) cloud_filtered->points.size ();
  // while (cloud_filtered->points.size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud (cloud_filtered);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0)
  //   {
  //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }
  //
  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<pcl::PointXYZ> extract;
  //   extract.setInputCloud (cloud_filtered);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);
  //
  //   // Get the points associated with the planar surface
  //   extract.filter (*cloud_plane);
  //   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  //
  //   // Remove the planar inliers, extract the rest
  //   extract.setNegative (true);
  //   extract.filter (*cloud_f);
  //   *cloud_filtered = *cloud_f;
  // }
  std::stringstream sss;
  sss << "cloud_cluster_after_filtter.pcd";
  writer.write<pcl::PointXYZ> (sss.str (), *cloud_filtered, false);
  writer.write<pcl::PointXYZ> ("cloud_cluster_after_filtter_2.pcd", *cloud_filtered2, false);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PassThrough<pcl::PointXYZ> pass_2;
  // pass.setInputCloud (cloud_filtered2);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (neg_z_plane, 0.8);
  // pass.filter (*cut_cloud_2);
  // writer.write<pcl::PointXYZ> ("cut_cloud_2.pcd", *cut_cloud_2, false);


  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered2);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered2);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered2->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.getHeight() << " data points." << std::endl;
    float x_min=9999;
    float y_min=9999;
    float z_min=9999;
    float x_max=-1;
    float y_max=-1;
    float z_max=-1;

    float down_y=9999;
    float up_y=-9999;
    float right_x=-9999;
    float left_x=9999;
    float pos_z=-9999;
    float neg_z=9999;

    for (int i=0; i<cloud_cluster->width;i++){
      float x=std::abs(cloud_cluster->points[i].x);
      float y=std::abs(cloud_cluster->points[i].y);
      float z=std::abs(cloud_cluster->points[i].z);

      x_min = std::min(x_min,x);
      y_min = std::min(y_min,y);
      z_min = std::min(z_min,z);

      x_max = std::max(x_max,x);
      y_max = std::max(y_max,y);
      z_max = std::max(z_max,z);
      // std::cout << "TEST" << cloud_cluster->points[i] << std::endl;

      left_x = std::min(left_x,cloud_cluster->points[i].x);
      down_y = std::min(down_y,cloud_cluster->points[i].y);
      neg_z = std::min(neg_z,cloud_cluster->points[i].z);

      right_x = std::max(right_x,cloud_cluster->points[i].x);
      up_y = std::max(up_y,cloud_cluster->points[i].y);
      pos_z = std::max(pos_z,cloud_cluster->points[i].z);
    }
    std::cout << "width  " << std::abs(left_x-right_x) << std::endl;
    std::cout << "height  " << std::abs(down_y-up_y) << std::endl;
    std::cout << "depth  " << std::abs(neg_z-pos_z) << std::endl;
    std::cout << "down_y  " << down_y << std::endl;
    std::cout << "up_y  " << up_y << std::endl;
    std::cout << "right_x  " << right_x << std::endl;
    std::cout << "left_x  " << left_x << std::endl;
    std::cout << "pos_z " << pos_z << std::endl;
    std::cout << "neg_z  " << neg_z << std::endl;
    std::stringstream ss;
    if ((x_max-x_min) < 0.4 && (y_max-y_min) < 0.3 && (z_max-z_min) < 1 ){
      std::cout << j << (pos_z_plane >= neg_z) << (neg_z_plane <= neg_z) << (left_x_plane <= left_x) << (right_x_plane >= right_x) << std::endl;
      if (pos_z_plane >= neg_z
        // && neg_z_plane <= neg_z
        && left_x_plane <= left_x
        && right_x_plane >= right_x){
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      } else{
        ss << "No-cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      }
    }
    j++;
  }

  return (0);
}
