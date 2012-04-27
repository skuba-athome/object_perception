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
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <std_msgs/String.h>

#define TOPIC_CONTROL "/cmd_state"

ros::Publisher pub;
int check = 0;
typedef std::pair<std::string, std::vector<float> > vfh_model;

std::vector<vfh_model> models;
flann::Matrix<int> k_indices;
flann::Matrix<float> k_distances;
flann::Matrix<float> data;
std::string kdtree_idx_file_name;
std::string training_data_h5_file_name ;
std::string training_data_list_file_name;
std::string name;

int k = 6;

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool loadFileList (std::vector<vfh_model> &models, const std::string &filename);


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
							int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);



/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool loadHist (const boost::filesystem::path &path, vfh_model &vfh);
