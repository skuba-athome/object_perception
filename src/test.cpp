#include "test.h"
#include <ros/package.h>
#include <string>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>

tf::TransformListener* listener;
std::string mani_frame = "/mani_link";
std::string robot_frame = "/base_link";
std::string camera_optical_frame = "/camera_rgb_optical_frame";

ros::Publisher chatter_pub;

void msg_cb(const std_msgs::String::ConstPtr& msg)
{
	//ROS_INFO("%s",msg->data.c_str());

	if(check)
	{
		ROS_INFO("lock process !!");
		return ;
	}

	name = msg->data.c_str();
	std::string name2=msg->data.c_str();
	name3=msg->data.c_str();
	if(name2=="apple juice"){
		name2 =  "applejuice";
		name = name2;
	}
	if(name2=="orange juice"){
		name2 =  "orangejuice";
		name = name2;
	}
	if(name2=="chocolate milk"){
		name2 =  "chocolatemilk";
		name = name2;
	}
	if(name2=="tomato sauce"){
		name2 =  "tamato";
		name = name2;
	}
	if(name2=="chicken noodies"){
		name2 =  "chickennoodies";
		name = name2;
	}
	if(name2=="veggie noodies"){
		name2 =  "veggienoodies";
		name = name2;
	}
if(name2=="energy drink"){
		name2 =  "energydrink";
		name = name2;
	}
	if(name2=="garlic sauce"){
		name2 =  "garlicsauce";
		name = name2;
	}
	if(name2=="tooth paste"){
		name2 =  "toothpaste";
		name = name2;
	}
	if(name2=="tomato sauce"){
		name2 =  "tomato";
		name = name2;
	}
	if(name2=="seven up"){
		name2 =  "sevenup";
		name = name2;
	}
	if(name2=="fresh discs"){
		name2 =  "freshdiscs";
		name = name2;
	}
if(name2=="beer can"){
		name2 =  "beercan";
		name = name2;
	}
if(name2=="peanut butter"){
		name2 =  "peanutbutter";
		name = name2;
	}
	std::string path = ros::package::getPath("objects");
	path=path+"/robocup2013";

	char idx[255];
	sprintf(idx,"%s/%s%s",path.c_str(),name.c_str(),".idx");
	//printf("%s\n",idx);
	kdtree_idx_file_name = idx;

	char h5[255];
	sprintf(h5,"%s/%s%s",path.c_str(),name.c_str(),".h5");
	training_data_h5_file_name = h5;
	//printf("%s\n",h5);

	char list[255];
	sprintf(list,"%s/%s%s",path.c_str(),name.c_str(),".list");
	training_data_list_file_name = list;
	//printf("%s\n",list);

	if (!boost::filesystem::exists (h5) || !boost::filesystem::exists (list))
	{
		pcl::console::print_error ("Could not find training data models files %s and %s!\n",
		training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
state = "no";
				std_msgs::String msg;
    				std::stringstream ss;

    				ss << state;
    				msg.data = ss.str();
    				ROS_INFO("%s", msg.data.c_str());
    				chatter_pub.publish(msg);
	    return ;
	}
	else
	{
		ROS_INFO("%s",name.c_str());
		loadFileList (models, training_data_list_file_name);
	    flann::load_from_file (data, training_data_h5_file_name, "training_data");
	    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n",(int)data.rows,training_data_h5_file_name.c_str(),training_data_list_file_name.c_str ());
	}

	check = 3;

}


Eigen::Vector4f plane_coeffs_in_robot_frame(Eigen::Vector4f coeffs_in)
{
	tf::StampedTransform transform;
	try{
		listener->lookupTransform(camera_optical_frame, robot_frame, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}	
		
	Eigen::Matrix4f T;
	pcl_ros::transformAsMatrix(transform,T);

	Eigen::MatrixXf coeffs(1,4); coeffs << coeffs_in(0), coeffs_in(1), coeffs_in(2), coeffs_in(3);
	Eigen::MatrixXf coeffs_out(1,4);
	coeffs_out = coeffs*T;
	Eigen::Vector4f plane_coeffs(coeffs_out(0,0),coeffs_out(0,1),coeffs_out(0,2),coeffs_out(0,3));

	return plane_coeffs;
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//ROS_INFO("%d",check);
	if(check)
	{
		pcl::PCDWriter writer;
		//ROS_INFO("regcognize !");
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
		vg.setLeafSize (0.003f, 0.003f, 0.005f);
		vg.filter (*cloud_filtered);
	//	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

		writer.write<pcl::PointXYZ> ("scence_f1.pcd", *cloud_filtered, false);

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());


		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		//seg.setMaxIterations (250);
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



	Eigen::Vector4f plane_coeffs(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
	Eigen::Vector4f plane_check = plane_coeffs_in_robot_frame(plane_coeffs);

	std::cout << "Table plane coeffs >>>\n"<<plane_check << std::endl;
	if(plane_check(2) > 0.85 && plane_check(3) < -0.45)
	{
		
		for(unsigned int n = 0; n < cloud_filtered->points.size(); n++)
		{
			Eigen::Vector4f tested_point(cloud_filtered->points[n].x, cloud_filtered->points[n].y, cloud_filtered->points[n].z, 1.0f);
			if(tested_point.dot(plane_coeffs) <= 0.0 || tested_point.dot(plane_coeffs) > 0.35) inliers->indices.push_back(n);	
		}
	
	}
	else if(plane_check(2) < -0.85 && plane_check(3) > 0.45)
	{
		
		for(unsigned int n = 0; n < cloud_filtered->points.size(); n++)
		{
			Eigen::Vector4f tested_point(cloud_filtered->points[n].x, cloud_filtered->points[n].y, cloud_filtered->points[n].z, 1.0f);
			if(tested_point.dot(plane_coeffs) >= 0.0 || tested_point.dot(plane_coeffs) < -0.35) inliers->indices.push_back(n);	
		}
	
	}



			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Write the planar inliers to disk
			extract.filter (*cloud_plane);
			//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			cloud_filtered = cloud_f;
		}



		writer.write<pcl::PointXYZ> ("scence_f2.pcd", *cloud_filtered, false);
		  // Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (2500);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);

		float x,y,z;
		//printf("debug : cluster \n");
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			x = 0.0f;
			y = 0.0f;
			z = 0.0f;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
				x+=cloud_filtered->points[*pit].x;
				y+=cloud_filtered->points[*pit].y;
				z+=cloud_filtered->points[*pit].z;
			}
			x/=cloud_cluster->points.size ();
			y/=cloud_cluster->points.size ();
			z/=cloud_cluster->points.size ();
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

			Eigen::Vector4f centroid;
			//pcl::compute3DCentroid(cloud_cluster,centroid);

			//printf("%.2f , %.2f , %.2f \n",centroid.);

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

			std::string path = ros::package::getPath("objects");
			path=path+"/robocup2013";
			std::stringstream ss;
			ss << path.c_str() <<"test.vfh.pcd";
			writer.write<pcl::VFHSignature308> (ss.str (), *vfhs, false);


			// Check if the tree index has already been saved to disk
			if (!boost::filesystem::exists (kdtree_idx_file_name))
			{
				pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
				state = "no";
				std_msgs::String msg;
    				std::stringstream ss;

    				ss << state;
    				msg.data = ss.str();
    				ROS_INFO("%s", msg.data.c_str());
    				chatter_pub.publish(msg);
				return ;
			}
			else
			{
				vfh_model histogram;
				loadHist (ss.str().c_str(), histogram );
				//printf("debug : segmentation fault !\n");
				char tmp[255];
				sprintf(tmp,"%s/%s.idx",path.c_str(),name.c_str()	);
				flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (tmp));
			    index.buildIndex ();
			    nearestKSearch (index, histogram, k, k_indices, k_distances);

			    // Output the results on screen
			   	pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, name.c_str() );
			    //for (int i = 0; i < k; ++i)
			    pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n",i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);

			    if(k_distances[0][0] < obj_threshold)
			    {
				geometry_msgs::PointStamped point_in, point_out;
				point_in.header = input->header;
				point_in.point.x = x;
				point_in.point.y = y;
				point_in.point.z = z;
    				try{
					listener->transformPoint(mani_frame,point_in,point_out);
				}
				catch (tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
				}	
					printf("%.2f %.2f %.2f  \n",point_out.point.x,point_out.point.y,point_out.point.z);
					// public value is exists object "S %x %y %z"
					printf("kinect %.2f %.2f %.2f  \n",z,x,y);
					char* str = new char[30];
 
					//publish to Main_Control
					//sprintf(str,"%s,%f,%f,%f",name.c_str(),z,x,y);
					sprintf(str,"%s,%.2f,%.2f,%.2f",name3.c_str(),point_out.point.x,point_out.point.y,point_out.point.z);
					state =str;
					check = 0;
					std_msgs::String msg;
   			 		std::stringstream ss;
	
		    		ss << state;
    				msg.data = ss.str();
    				ROS_INFO("%s", msg.data.c_str());
    				chatter_pub.publish(msg);
					return ;
			    }
			}

		 }
		// public value is not exists "N ..."
		check-- ;
		if(check == 0)
		{
			//state = 'N';
			state = "no";
			std_msgs::String msg;
    		std::stringstream ss;

    		ss << state;
    		msg.data = ss.str();
    		ROS_INFO("%s", msg.data.c_str());
    		chatter_pub.publish(msg);
		}
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

	//chatter_pub = n.advertise<std_msgs::String>("objects_to_voice", 1000);
	chatter_pub = n.advertise<std_msgs::String>("/object/output", 1000);
	ros::Rate loop_rate(10);
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("input", 1);

	//ros::Subscriber sub2 = n.subscribe(TOPIC_CONTROL, 1, msg_cb);
	ros::Subscriber sub2 = n.subscribe("/object/search", 1, msg_cb);
	
	listener = new tf::TransformListener();
/*
	// Spin
	while (ros::ok())
  	{
    	    	ros::spinOnce();
    	loop_rate.sleep();
  	}
*/
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
