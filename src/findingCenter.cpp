#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <my_pcl/cropped_object.h>
//#include <my_pcl/cropped_msg.h>
//#include <object_perception/cropped_msg.h>
//#include <pcl_ros/transforms.h>
//#include <pcl/ros/conversions.h>
//#include <pcl/point_types.h>
#include <object_perception/classifyObject.h>
#include <manipulator/isManipulable.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <pcl17_ros/transforms.h>
#include <pcl17/ros/conversions.h>
#include <pcl17/ModelCoefficients.h>
#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/filters/extract_indices.h> 
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/kdtree/kdtree.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/segmentation/extract_clusters.h>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <algorithm>
#include <tf/transform_listener.h>

using namespace std;

//#define DEPTH_LIMIT 0.86
#define DEPTH_LIMIT 1.25
#define ACCEPTED_AREA 200
//#define PLANE_HEIGHT -1.5
#define PLANE_HEIGHT 0.5
#define PLANE_LEFT -0.35
#define PLANE_RIGHT 0.35
//IR 580
//RGB 525
#define FOCAL_LENGTH 525
#define CENTER_IMAGE_X 320
#define CENTER_IMAGE_Y 240
#define TUNED_H_DISTANCE_TOP_LEFT 10
#define TUNED_H_DISTANCE_BOTTOM_RIGHT 10
#define TUNED_V_DISTANCE_TOP_LEFT -10
#define TUNED_V_DISTANCE_BOTTOM_RIGHT 20
#define ARM_RAIDUS 1.15
int maxArea=0;
float pixel_x,pixel_y;
float pos_x,pos_y,pos_z;
cv::Mat img;
int object_position_world[20][3];
bool isReach[20];
vector<std::string> fileName;
int objectCentroidWorld[20][3];


std::string frameId;
std::string robot_frame = "base_link";
std::string pan_frame = "pan_link";
//std::string robot_frame = "/base_link";
//std::string pan_frame = "/pan_link";

//convert to world frame service 
//ros::ServiceClient convertClient;
//object_perception::ConvertToWorld convertSrv;

//verification service 
ros::ServiceClient classifyClient;
object_perception::classifyObject classifySrv;

//is reachable service 
ros::ServiceClient isManipulableClient;
manipulator::isManipulable isManipulatableSrv;

tf::TransformListener* listener;
double timeStamp=0;


bool isObjectReachable(float x,float y,float z){
	cout << "-------------------in isObjectReachable method-------------------------" << endl;
	cout << "centroid of object in kinect :  (x,y,z) = " << x << " " << y << " " << z << endl;
	float xWorld,yWorld,zWorld;
	//char* robot_frame = "";

	//try{
		//pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_obj (new pcl17::PointCloud<pcl17::PointXYZ>);
		pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_obj (new pcl17::PointCloud<pcl17::PointXYZ>);

//		PointCloudT::Ptr cloud (new PointCloudT);
//		pcl::fromROSMsg(*cloud_in,*cloud);
		
		geometry_msgs::PointStamped kinect_point;
		geometry_msgs::PointStamped base_point;
//		geometry_msgs::Point point;
//		std_msgs::Header header;

		kinect_point.header.frame_id = "camera_rgb_optical_frame";
		kinect_point.header.stamp = ros::Time();
		kinect_point.point.x = x;
		kinect_point.point.y = y;
		kinect_point.point.z = z;

		cout << "robot_frame = " << robot_frame << ", frameId = " << frameId << endl;
		//listener->transformPoint(robot_frame,ros::Time::now(),kinect_point,pan_frame, base_point);
		listener->transformPoint(robot_frame,kinect_point,base_point);


		cout << "returned pointStamped from transformPoint function : (" << base_point.point.x << "," << base_point.point.y << "," << base_point.point.z << ")" <<endl;

//		listener->waitForTransform(robot_frame, frameId, ros::Time::now(), ros::Duration(1.0));
		//pcl17_ros::transformPointCloud("base_link", *cloud, *cloud_obj, *listener);

//		xWorld = cloud_obj->begin()->x;
//		yWorld = cloud_obj->begin()->y;
//		zWorld = cloud_obj->begin()->z;
		//xWorld = 1.3;
		//yWorld = 0.3;
		//zWorld = 0.3;
//	}
//	catch(tf::TransformException& ex){
//		//ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", cloud_in->header.frame_id.c_str(),pan_frame.c_str(),ex.what());
//		ROS_ERROR("Received an exception trying to transform a point.");// from %s to %s: %s", cloud_in->header.frame_id.c_str(),pan_frame.c_str(),ex.what());
//	}
//
	isManipulatableSrv.request.x = xWorld;
	isManipulatableSrv.request.y = yWorld;
	isManipulatableSrv.request.z = zWorld;
	cout << "WORLD " << xWorld << " " << yWorld << " " << zWorld << endl;

	if(isManipulableClient.call(isManipulatableSrv)){
		bool tmp_ = (bool)isManipulatableSrv.response.isManipulable;
		cout << "response from isManipulable server : " << tmp_ << endl;
		cout << "--------------------------------------------" << endl;
	}
	else
		ROS_ERROR("Failed to call isManipulable service");

	return true;
}

ros::Publisher pub,pubObjectNum,center_pub;
static pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_pcl (new pcl17::PointCloud<pcl17::PointXYZ>);

void depthCB(const sensor_msgs::PointCloud2& cloud) {
	if ((cloud.width * cloud.height) == 0)
		return; //return if the cloud is not dense!
	try {
		frameId = cloud.header.frame_id;
		//timeStamp = cloud.header.frame_id;
		pcl17::fromROSMsg(cloud, *cloud_pcl);
		//ROS_INFO("Get PointCloud size : %d",cloud.width*cloud.height);
//		cout << "point cloud get" << endl;
	} catch (std::runtime_error e) {
		ROS_ERROR_STREAM("Error message: " << e.what());
	}
}

void getObjectPoint(){
	 pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_f (new pcl17::PointCloud<pcl17::PointXYZ>)    ,cloud_tmp (new pcl17::PointCloud<pcl17::PointXYZ>);
	//Read in the cloud data
	//pcl17::PCDReader reader;
	//reader.read ("1389374538.210053381.pcd", *cloud);
	
	int reachableCount=0;
	std::stringstream center_ss;
	cloud = cloud_pcl;

	vector<int> compression_params; //vector that stores the compression parameters of the image

	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique

	compression_params.push_back(98); //specify the compression quality

	//bool bSuccess_ = imwrite("first.jpg", img, compression_params); //write the image to file
	bool bSuccess_ = imwrite("/run/shm/object_perception/first.jpg", img, compression_params); //write the image to file
	
	   //pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_f (new pcl17::PointCloud<pcl17::PointXYZ>);
	   
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
	pcl17::PCDWriter writer;
	writer.write<pcl17::PointXYZ> ("/run/shm/object_perception/first.pcd", *cloud, false); 


	for (pcl17::PointCloud<pcl17::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it){
		float x = it->x;
		float y = it->y;
		float z = it->z;
		//countOut++;
		if( sqrt(x*x+y*y+z*z) < ARM_RAIDUS){
			//countIn++;
			cloud_tmp->push_back(pcl17::PointXYZ(it->x,it->y,it->z));
		}
	}
	

	// Create the filtering object: downsample the dataset using a leaf size of 1cm

	pcl17::VoxelGrid<pcl17::PointXYZ> vg;
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_filtered (new pcl17::PointCloud<pcl17::PointXYZ>);
	vg.setInputCloud (cloud_tmp);
	//adjust?
	//vg.setLeafSize (0.001f, 0.001f, 0.001f);
	vg.setLeafSize (0.002f, 0.002f, 0.002f);
	//vg.setLeafSize (0.1f, 0.1f, 0.1f);
	vg.filter (*cloud_filtered);

	std::cout << "PointCloud(cloud) before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

	//Create the segmentation object for the planar model and set all the parameters


//	std::cout << "PointCloud after(cloud_filtered = cloud) filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 



	writer.write<pcl17::PointXYZ> ("/run/shm/object_perception/cloud_filtered.pcd", *cloud_filtered, false); 

	pcl17::SACSegmentation<pcl17::PointXYZ> seg;
	pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices);
	pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients);
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_plane (new pcl17::PointCloud<pcl17::PointXYZ> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl17::SACMODEL_PLANE);
	seg.setMethodType (pcl17::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int i=0, nr_points = (int) cloud_filtered->points.size ();
	int j =0;
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	//while (cloud_filtered->points.size () > 0.1 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);

		std::vector< float > coef = coefficients->values;
		for(int j=0;j<coef.size();j++){
			ROS_INFO("%f ",coef.at(j));
		}
		ROS_INFO("\n");

		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl17::ExtractIndices<pcl17::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;


		std::stringstream plane_name;
		plane_name << "/run/shm/object_perception/segmenged_plane" << j << ".pcd";
		writer.write<pcl17::PointXYZ> (plane_name.str (), *cloud_plane, false); 
		j++;
	}

	// Creating the KdTree object for the search method of the extraction

	pcl17::search::KdTree<pcl17::PointXYZ>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl17::PointIndices> cluster_indices;
	pcl17::EuclideanClusterExtraction<pcl17::PointXYZ> ec;
	//ec.setClusterTolerance (0.02); // 2cm
	ec.setClusterTolerance (0.02); // 2cm
	//ec.setMinClusterSize (ACCEPTED_AREA);
	ec.setMinClusterSize (ACCEPTED_AREA);
	//ec.setMinClusterSize (200);
	//ec.setMaxClusterSize (25000);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_center (new pcl17::PointCloud<pcl17::PointXYZ>);

	//cout << "-------------------after ece -------------------------" << endl;

	int objectNum=0;
	bool reachable;
	j = 0;


	cout << "cluster_indeces.size() = " << cluster_indices.size() << endl;

	//cout << "-------------------1-------------------------" << endl;
	reachableCount=0;
	do{
		//cout << "-------------------2-------------------------" << endl;
		//cout << "cluster_indece.begin() " << cluster_indices.begin() << endl;
		reachable=true;
		for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{	
			//cout << "-------------------3-------------------------" << endl;
			float x=0,y=0,z=0;
			float pixel_x_max = 0, pixel_y_max = 0, pixel_x_min = 640, pixel_y_min = 480;

			pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_cluster (new pcl17::PointCloud<pcl17::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
				float tmpX,tmpY;
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
				x += cloud_filtered->points[*pit].x;
				y += cloud_filtered->points[*pit].y;
				z += cloud_filtered->points[*pit].z;

				tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH/cloud_filtered->points[*pit].z + CENTER_IMAGE_X;
				tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH/cloud_filtered->points[*pit].z + CENTER_IMAGE_Y;

				if(tmpX > pixel_x_max) pixel_x_max = tmpX;
				if(tmpY > pixel_y_max) pixel_y_max = tmpY;
				if(tmpX < pixel_x_min) pixel_x_min = tmpX;
				if(tmpY < pixel_y_min) pixel_y_min = tmpY;

			}
			//cout << "-------------------after first loop-------------------------" << endl;

			pixel_x_min = pixel_x_min > 0 ? pixel_x_min : 0;
			pixel_y_min = pixel_y_min > 0 ? pixel_y_min : 0;
			pixel_x_max = pixel_x_max < 2*CENTER_IMAGE_X ? pixel_x_max : 2*CENTER_IMAGE_X;
			pixel_y_max = pixel_y_max < 2*CENTER_IMAGE_Y ? pixel_y_max : 2*CENTER_IMAGE_Y;

			int size = std::distance(it->indices.begin(), it->indices.end());
			x/=size;
			y/=size;
			z/=size;

			//if(z > DEPTH_LIMIT || x < PLANE_LEFT || x > PLANE_RIGHT || y < PLANE_RIGHT)
		//	if(z > DEPTH_LIMIT || x < PLANE_LEFT || x > PLANE_RIGHT)
		//		continue;


			if(z > DEPTH_LIMIT || x < PLANE_LEFT || x > PLANE_RIGHT)
				continue;

			//cout << "-------------------before isObjectReachable method-------------------------" << endl;
			//
			if(isObjectReachable(x,y,z))
				reachableCount++;


	//		if(size > maxArea){
	//			maxArea = size;
	//			pos_x = x;
	//			pos_y = y;
	//			pos_z = z;
	//		}
	//		else
	//			continue;

			printf("(%f,%f,%f)\n",x,y,z);

			pixel_x = x*FOCAL_LENGTH/z + CENTER_IMAGE_X;
			pixel_y = y*FOCAL_LENGTH/z + CENTER_IMAGE_Y;

			center_ss << pixel_x << " " << pixel_y << " ";

			cv::Mat m = img.clone();
			IplImage* iplImage = new IplImage(m);
			//IplImage* iplImage = new IplImage(m);
			//img.copyTo(iplImage);
			//iplImage = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3); 
			//iplImage = 


//use thie one
			//printf("pixel_x_min : %f, pixel_y_min : %f, pixel_x_max : %f, pixel_y_max : %f\n",pixel_x_min,pixel_y_min,pixel_x_max,pixel_y_max);
			//printf("top left : (%f,%f)\n",max(pixel_x_min-TUNED_H_DISTANCE_TOP_LEFT,(float)0),max(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT,(float)0));
			//printf("bottom right : (%f,%f)\n",min(pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_X)),min(pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_Y)));
			//ROS_INFO("write picture.%d with (%f,%f,%f,%f) iplimage->size() : %d img.size() : %d",j,pixel_x_min,pixel_y_min,pixel_x_max,pixel_y_max,iplImage->width*iplImage->height,img.rows*img.cols);

			cvSetImageROI(iplImage,cv::Rect(max(pixel_x_min-TUNED_H_DISTANCE_TOP_LEFT,(float)0),
										    max(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT,(float)0),
											min(pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_X)),
											min(pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_Y))));



			std::stringstream ss_;

//use this one
			ss_ << "/run/shm/object_perception/picture" << j << ".jpg";

//use this one
			bool bSuccess = imwrite(ss_.str(), cv::Mat(iplImage), compression_params); //write the image to file



//use this one
			//------------------------------------------------------

			char comm[1000];
			//sprintf(comm,"/home/skuba/skuba_athome/object_perception/bin/extractSURF /home/skuba/skuba_athome/object_perception/picture%d.jpg /home/skuba/skuba_athome/object_perception/feature%d",j,j);
			sprintf(comm,"/home/skuba/skuba_athome/object_perception/bin/extractSURF /run/shm/object_perception/picture%d.jpg /run/shm/object_perception/feature%d",j,j);
			system(comm);

			//------------------------------------------------------



//use this one
			//------------------------------------------------------
			//cv_bridge::CvImage cv_ptr;

			//ROS_INFO("BEFORE ASSIGN cv_ptr->image = cv::Mat(iplImage)\n");
			//cv_ptr.image = cv::Mat(iplImage);
			//ROS_INFO("AFTER ASSIGN cv_ptr->image = cv::Mat(iplImage)\n");
			//------------------------------------------------------

			cv_bridge::CvImage cv_ptr;
			//my_pcl_tutorial::cropped_object co;

			//ROS_INFO("BEFORE co.img = *(cv_ptr->toImageMsg())\n");
			//co.img = *(cv_ptr.toImageMsg());
			//ROS_INFO("AFTER co.img = *(cv_ptr->toImageMsg())\n");
			//
			//sensor_msgs::Image img_;
			//img_ = cv_ptr->toImageMsg();
			//co.img = cv_ptr->toImageMsg();

			geometry_msgs::Vector3 vector_;
			vector_.x = pixel_x;
			vector_.y = pixel_y;

			//co.vector = vector_;



			//object_perception::cropped_msg cms[20];
			//cms[j].vector = vector_;
			//object_perception::cropped_msg cm;
			//cm.vector = vector_;


			std::stringstream sf;
			sf << "/run/shm/object_perception/feature" << j;

			fileName.push_back(sf.str());
			printf("fileName = %s\n",fileName[j].c_str());
			//have to change to centroid, now it's the center of object in 2d domain
			objectCentroidWorld[j][0] = pixel_x;
			objectCentroidWorld[j][1] = pixel_y;
			//objectCentroidWorld[j][2] = z;
			


			//cm.filePath = sf.str();
			//cms[j].filePath = sf.str();
			//sending message, must be changed!
			//cropped_msg_pub.publish(cm);


			//cv::imshow("window",cv::Mat(iplImage));

			//result position
			//ROS_INFO("(pos_x : %f, pos_y : %f, pos_z : %f\n",pos_x,pos_y,pos_z);
			ROS_INFO("(object's point in RGB (%f,%f)\n",pixel_x,pixel_y);
			ROS_INFO("TOPLEFT : (%f,%f) TOP_RIGHT : (%f,%f)\n",pixel_x_min,pixel_y_min,pixel_x_max,pixel_y_max);

			//pcl17::PointCloud<pcl17::PointXYZ> point;

			//pcl17::PointXYZ point;
			//point.x = x;  point.y = y;  point.z = z;
			//cloud_center->push_back(point);
			
			cloud_center->push_back(pcl17::PointXYZ(x,y,z));

			//cloud_center->push_back(new pcl17::PointXYZ(x,y,z));
			ROS_INFO("center of clustering no.%d : (%f,%f,%f) \n",j,x,y,z);

			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << "/run/shm/object_perception/cloud_cluster_" << j << ".pcd";

//------------------------------------cloud_cluster -------------------------
			writer.write<pcl17::PointXYZ> (ss.str (), *cloud_cluster, false); 
			j++;
		}

		/*
		if((float)reachableCount/j<0.7){
			//demo
			lumyai_navigation_msgs::NavGoalMsg goal_pose;
			goal_pose.text_msg = "closering";
			goal_pose.ref_frame = "relative";
			goal_pose.pose2d.x = 0.05;
			goal_pose.pose2d.y = 0;
			goal_pose.pose2d.theta = 0;
			goal_pub.publish(goal_pose);
			reachable = false;
			//delay, waiting for reaching the destination and for new frame of point cloud
		}
		*/
	//}while((float)reachableCount/j<0.7);
	}while(false);

	cout << "isReachable : " << reachableCount << endl;



	if(true){
		for(int k=0;k<j;k++){
			classifySrv.request.filepath = fileName[k];
			//classifySrv.request.x = objectCentroidWorld[k][0];
			//classifySrv.request.y = objectCentroidWorld[k][1];
			//classifySrv.request.z = objectCentroidWorld[k][2];
			cout << "--------------------------" << fileName[k] << endl;

			if(classifyClient.call(classifySrv))
				cout << "response from server : " << classifySrv.response.objectIndex << endl;
		}
	}
	else{
		//do nothing, no object can manipulate in this range.
	}
	
	
	

	std_msgs::String string_msg;
	string_msg.data = center_ss.str();
	center_pub.publish(string_msg);

	std::stringstream tmpSs;
	tmpSs <<  j << "";
	std_msgs::String msg_;
	msg_.data = tmpSs.str();
	pubObjectNum.publish(msg_);
//
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr enlarged_cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
	//create center of object cloud
	for(size_t t = 0; t < cloud_center->size(); t++){
		for(float i=-0.007;i<=0.007;i+=0.001){
			for(float j=-0.007;j<=0.007;j+=0.001){
				for(float k=-0.007;k<=0.007;k+=0.001){
					enlarged_cloud->push_back(pcl17::PointXYZ(cloud_center->points[t].x+i,cloud_center->points[t].y+j,cloud_center->points[t].z+k));
				}
			}
		}
	}


//--------------------------------------------------centerOfObject-------------------------
	writer.write<pcl17::PointXYZ> ("/run/shm/object_perception/centerOfObject.pcd", *enlarged_cloud, false); 
//
	if(maxArea == 0){
		ROS_ERROR("No object in this range.");
		return ; 
	}

	//pixel_x = x*FOCAL_LENGTH/z + CENTER_IMAGE_X;
	//pixel_y = y*FOCAL_LENGTH/z + CENTER_IMAGE_Y;

	geometry_msgs::Vector3 vector;
	vector.x = pos_x*FOCAL_LENGTH/pos_z + CENTER_IMAGE_X;
	vector.y = pos_y*FOCAL_LENGTH/pos_z + CENTER_IMAGE_Y;
	vector.z = 0;

	//std::string cmd(s.str());
	//std_msgs::String msg;
	//msg.data = s.str();
	//pub.publish(cmd.data());
	//pub.publish(msg.data.c_str());

	//std_msgs::String msg;

	//std::stringstream ss;
	//ss << "P1(" << pos_x << "," << pos_y <<")";
	//msg.data = ss.str();

	//ROS_INFO("%s", msg.data.c_str());

	pub.publish(vector);

//printf(" pixel : %s",center_ss.str())
	std::cout << "pixel : " << center_ss.str();



	/*
	   pixel_x = pos_x*FOCAL_LENGTH/pos_z + CENTER_IMAGE_X;
	   pixel_y = pos_y*FOCAL_LENGTH/pos_z + CENTER_IMAGE_Y;

	//result position
	ROS_INFO("(pos_x : %f, pos_y : %f, pos_z : %f\n",pos_x,pos_y,pos_z);
	ROS_INFO("(object's point in RGB (%f,%f)\n",pixel_x,pixel_y);
	*/
	//ROS_INFO("(object's point (%f,%f,%f)\n",vector.x,vector.y,vector.z);

	//pub.publish(vector);
}


void localizeCb(const std_msgs::String::ConstPtr& msg){
	getObjectPoint();
}


void imageColorCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img = cv_ptr->image;
		//ROS_INFO("Get image size : %d",img.rows*img.cols);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}


int main (int argc, char** argv)
{

	ros::init(argc,argv,"findCenter");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	image_transport::ImageTransport it_(nh);
	image_transport::Subscriber sub_imageColor;

	pub = n.advertise<geometry_msgs::Vector3>("largest_point", 1000);
	pubObjectNum = n.advertise<std_msgs::String>("object_number", 1000);
	//cropped_msg_pub = n.advertise<object_perception::cropped_msg>("cropped_msg", 100);
	center_pub = n.advertise<std_msgs::String>("center_pcl_object", 1000);
	//goal_pub = n.advertise<lumyai_navigation_msgs::NavGoalMsg>("/follow/point", 1);
	ros::Subscriber sub = n.subscribe("/camera/depth_registered/points",1,depthCB);
	//ros::Subscriber sub = n.subscribe("/camera/depth/points",1,depthCB);
	ros::Subscriber sub_ = n.subscribe("localization",1,localizeCb);
	sub_imageColor = it_.subscribe("/camera/rgb/image_color", 1, imageColorCb);

	classifyClient = n.serviceClient<object_perception::classifyObject>("classifyObject");
	object_perception::classifyObject classifySrv;

	isManipulableClient = n.serviceClient<manipulator::isManipulable>("isManipulable");
	manipulator::isManipulable isManipulatableSrv;

	listener = new tf::TransformListener();
	ros::spin();

	return (0);
}
