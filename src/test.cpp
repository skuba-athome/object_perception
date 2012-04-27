#include "test.h"

void msg_cb(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s",msg->data.c_str());


	name = msg->data.c_str();

	char idx[20];
	sprintf(idx,"%s%s",msg->data.c_str(),".idx");
	printf("%s\n",idx);
	kdtree_idx_file_name = idx;

	char h5[20];
	sprintf(h5,"%s%s",msg->data.c_str(),".h5");
	training_data_h5_file_name = h5;
	printf("%s\n",h5);

	char list[20];
	sprintf(list,"%s%s",msg->data.c_str(),".list");
	training_data_list_file_name = list;
	printf("%s\n",list);

	if (!boost::filesystem::exists (h5) || !boost::filesystem::exists (list))
	{
		pcl::console::print_error ("Could not find training data models files %s and %s!\n",
			training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	    return ;
	}
	else
	{
		ROS_INFO("%s",msg->data.c_str());
		loadFileList (models, training_data_list_file_name);
	    flann::load_from_file (data, training_data_h5_file_name, "training_data");
	    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n",
	        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	}

	check = 1;

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	if(check)
	{
		ROS_INFO("regcognize !");
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
		pcl::fromROSMsg(*input , *cloud);

	//	std::cout << cloud->height << " " << cloud->width << std::endl;

		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.4, 2.0);
		pass.filter (*cloud);
	//	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		vg.setInputCloud (cloud);
		vg.setLeafSize (0.0075f, 0.0075f, 0.01f);
		vg.filter (*cloud_filtered);
	//	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PCDWriter writer;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (250);
		seg.setDistanceThreshold (0.01);

		int i=0, nr_points = (int) cloud_filtered->points.size ();
		while (cloud_filtered->points.size () > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Write the planar inliers to disk
			extract.filter (*cloud_plane);
			std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			cloud_filtered = cloud_f;
		}

		  // Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.05); // 2cm
		ec.setMinClusterSize (200);
		ec.setMaxClusterSize (2500);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

			//============================== vfh =====================================

			// normal estimate
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud (cloud_cluster);
			// Use all neighbors in a sphere of radius 3cm
			ne.setRadiusSearch (0.03);
			// Compute the features
			ne.compute (*normals);

			// Create the VFH estimation class, and pass the input dataset+normals to it

			pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
			vfh.setInputCloud(cloud_cluster);
			vfh.setInputNormals(normals);

			// alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

			// Create an empty kdtree representation, and pass it to the FPFH estimation object.
			// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
			//vfh.setSearchMethod (tree);


			// Output datasets
			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

			// Compute the features
			vfh.compute (*vfhs);

			std::stringstream ss;
			ss << "test.vfh.pcd";
			writer.write<pcl::VFHSignature308> (ss.str (), *vfhs, false);

			// Check if the tree index has already been saved to disk
			if (!boost::filesystem::exists (kdtree_idx_file_name))
			{
				pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
				return ;
			}
			else
			{
				vfh_model histogram;
				loadHist ("test.vfh.pcd", histogram );

				flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
			    index.buildIndex ();
			    nearestKSearch (index, histogram, k, k_indices, k_distances);

			    // Output the results on screen
			    pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, name.c_str() );
			    for (int i = 0; i < k; ++i)
			      pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n",
			          i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
			}

		 }
		check = 0;

		writer.write<pcl::PointXYZ> ("scence.pcd", *cloud, false);
	}

}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh,n;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("input", 1);

	ros::Subscriber sub2 = n.subscribe(TOPIC_CONTROL, 1, msg_cb);

	// Spin
	ros::spin ();
}



bool loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}


inline void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
							int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

bool loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    sensor_msgs::PointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; int idx;
    const std::string temp_str = path.string();
    r.readHeader(temp_str, cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (pcl::InvalidConversionException e)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <sensor_msgs::PointField> fields;
  getFieldIndex (point, "vfh", fields);

  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}
