#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_recognition/classifyObject.h>
#include <object_recognition/verifyObject.h>
#include <manipulator/isManipulable.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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

#include <object_recognition/Object.h>
#include <object_recognition/ObjectContainer.h>

#include <unistd.h>
using namespace std;

#define DEPTH_LIMIT 0.86
//#define DEPTH_LIMIT 1.0
#define ACCEPTED_AREA 300
//#define PLANE_HEIGHT -1.5
//#define PLANE_HEIGHT 0.7
#define PLANE_LEFT 0.35
#define PLANE_RIGHT -0.35
//IR 580
//RGB 525
#define FOCAL_LENGTH 525
//#define FOCAL_LENGTH_X 814.033512
//#define FOCAL_LENGTH_Y 815.46674
//



//for webcam resolution 640x480
//#define CENTER_IMAGE_X 320
//#define CENTER_IMAGE_Y 240

//#define CALIBRATED_CENTER_IMAGE_X 313.73619
//#define CALIBRATED_CENTER_IMAGE_Y 254.26251


//#define TUNED_H_DISTANCE_TOP_LEFT 10
//#define TUNED_H_DISTANCE_BOTTOM_RIGHT 10
//#define TUNED_V_DISTANCE_TOP_LEFT 5
//#define TUNED_V_DISTANCE_BOTTOM_RIGHT 40

//#define TUNED_H_DISTANCE_TOP_LEFT -30
//#define TUNED_H_DISTANCE_BOTTOM_RIGHT 35
//#define TUNED_V_DISTANCE_TOP_LEFT 35
//#define TUNED_V_DISTANCE_BOTTOM_RIGHT 80



//#define TUNED_H_DISTANCE_TOP_LEFT 0
//#define TUNED_H_DISTANCE_BOTTOM_RIGHT 0
//#define TUNED_V_DISTANCE_TOP_LEFT 0
//#define TUNED_V_DISTANCE_BOTTOM_RIGHT 0
//#define ARM_RAIDUS 1.1

double CENTER_IMAGE_X;
double CENTER_IMAGE_Y;
double CALIBRATED_CENTER_IMAGE_X;
double CALIBRATED_CENTER_IMAGE_Y;
double FOCAL_LENGTH_X;
double FOCAL_LENGTH_Y;
double TUNED_H_DISTANCE_TOP_LEFT;
double TUNED_H_DISTANCE_BOTTOM_RIGHT;
double TUNED_V_DISTANCE_TOP_LEFT;
double TUNED_V_DISTANCE_BOTTOM_RIGHT;
double ARM_RAIDUS;



int maxArea=0;
float pixel_x,pixel_y;
float pos_x,pos_y,pos_z;
cv::Mat img,img_;
float object_position_world[20][3];
bool isReach[20];
float objectCentroidWorld[20][3];
bool isObjectManipulable[20];
float PLANE_HEIGHT = 0.75;
bool waiting_cloud_flags = false;
bool ready_flag = false;
void getObjectPoint();


std::string frameId;
std::string robot_frame = "base_link";
std::string pan_frame = "pan_link";
std::string logitech_frame = "logitech_cam";

std::string category;
//std::string robot_frame = "/base_link";
//std::string pan_frame = "/pan_link";

//convert to world frame service 
//ros::ServiceClient convertClient;
//object_perception::ConvertToWorld convertSrv;

//verification service 
ros::ServiceClient classifyClient;
object_recognition::classifyObject classifySrv;

ros::ServiceClient verifyClient;
object_recognition::verifyObject verifySrv;

//is reachable service 
ros::ServiceClient isManipulableClient;
manipulator::isManipulable isManipulatableSrv;

tf::TransformListener* listener;
ros::Time timeStamp,timeStamp_;

ros::Publisher pub,pubObjectNum,center_pub,pub_detectedObject;
static pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_pcl (new pcl17::PointCloud<pcl17::PointXYZ>);

bool isObjectReachable(float x,float y,float z,float* objectWorldX,float* objectWorldY,float* objectWorldZ){
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
		geometry_msgs::PointStamped logitech_point;
//		geometry_msgs::Point point;
//		std_msgs::Header header;

		//kinect_point.header.frame_id = "camera_rgb_optical_frame";
		//kinect_point.header.stamp = timeStamp_;
		//kinect_point.point.x = x;
		//kinect_point.point.y = y;
		//kinect_point.point.z = z;
        
		logitech_point.header.frame_id = logitech_frame;
		logitech_point.header.stamp = timeStamp_;
		logitech_point.point.x = x;
		logitech_point.point.y = y;
		logitech_point.point.z = z;

		//cout << "robot_frame = " << robot_frame << ", frameId = " << frameId << endl;
		//listener->transformPoint(robot_frame,ros::Time::now(),kinect_point,pan_frame, base_point);
		//listener->transformPoint(robot_frame,kinect_point,base_point);
		listener->transformPoint(robot_frame,logitech_point,base_point);

		cout << "returned pointStamped from transformPoint function : (" << base_point.point.x << "," << base_point.point.y << "," << base_point.point.z << ")" <<endl;

		*objectWorldX = base_point.point.x;
		*objectWorldY = base_point.point.y;
		*objectWorldZ = base_point.point.z;

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
	isManipulatableSrv.request.x = base_point.point.x;
	isManipulatableSrv.request.y = base_point.point.y;
	isManipulatableSrv.request.z = base_point.point.z;
	//cout << "WORLD " << xWorld << " " << yWorld << " " << zWorld << endl;

	if(isManipulableClient.call(isManipulatableSrv)){
		bool tmp_ = (bool)isManipulatableSrv.response.isManipulable;
		cout << "response from isManipulable server : " << tmp_ << endl;
		cout << "--------------------------------------------" << endl;
        return tmp_;
	}
	else
		ROS_ERROR("Failed to call isManipulable service");

	return false;
}


void depthCB(const sensor_msgs::PointCloud2& cloud) {

    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_tmp (new pcl17::PointCloud<pcl17::PointXYZ>);
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_tmp2 (new pcl17::PointCloud<pcl17::PointXYZ>);
    //PointCloudT::Ptr cloud_tmp (new PointCloudT);

    if(ready_flag == true){
        ready_flag = false;
        if ((cloud.width * cloud.height) == 0)
            return; //return if the cloud is not dense!
        try {
            //frameId = cloud.header.frame_id;
            timeStamp = cloud.header.stamp;
            //pcl17::fromROSMsg(cloud, *cloud_pcl);
            pcl17::fromROSMsg(cloud, *cloud_tmp);
            int n = 0;

            for (pcl17::PointCloud<pcl17::PointXYZ>::iterator it = cloud_tmp->begin(); it != cloud_tmp->end(); ++it){
                float x = it->x;
                float y = it->y;
                float z = it->z;
                //cout << "(" << x << "," << y << "," << z << ")" << endl;
                //if( x < DEPTH_LIMIT && y < PLANE_LEFT && y > PLANE_RIGHT && z > PLANE_HEIGHT){
                if( x < DEPTH_LIMIT && y < PLANE_LEFT && y > PLANE_RIGHT && z > PLANE_HEIGHT){
                    cloud_tmp2->push_back(pcl17::PointXYZ(it->x,it->y,it->z));
                    n++;
                }
                //if( sqrt(x*x+y*y+z*z) < ARM_RAIDUS){
                //    cloud_tmp2->push_back(pcl17::PointXYZ(it->x,it->y,it->z));
                //}
            }
            cloud_tmp2->header = cloud_tmp->header;
            cout << "first cloud size : " <<cloud_tmp->width*cloud_tmp->height << "modified cloud size : " << n << endl;

            //        cloud_tmp2->width = 1;
            //        cloud_tmp2->height = n;
            //cout << "width : " << cloud_tmp2->width << " height : " << n << endl;
            //cout << "frame_id " << cloud.header.frame_id << endl;
            listener->waitForTransform(logitech_frame, cloud.header.frame_id, cloud.header.stamp, ros::Duration(1.0));
            //listener->waitForTransform(logitech_frame, cloud.header.frame_id, cloud.header.stamp, ros::Duration(2.0));
            //cout << "before transformPointCloud" << endl;
            pcl17_ros::transformPointCloud(logitech_frame, *cloud_tmp2, *cloud_pcl, *listener);

            getObjectPoint();
            //cout << "cloud_pcl->header.frame_id" << cloud_pcl->header.frame_id << endl;

            //ROS_INFO("Get PointCloud size : %d",cloud.width*cloud.height);
            //	cout << "point cloud get" << endl;
            } catch (std::runtime_error e) {
                ROS_ERROR_STREAM("Error message: " << e.what());
            }
        }
}


void getObjectPoint(){

    timeStamp_ = timeStamp;
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_f (new pcl17::PointCloud<pcl17::PointXYZ>)    ,cloud_tmp (new pcl17::PointCloud<pcl17::PointXYZ>);
	//Read in the cloud data
	//pcl17::PCDReader reader;
	//reader.read ("1389374538.210053381.pcd", *cloud);
	
	int reachableCount=0,objectNumber=0;
	std::stringstream center_ss;
	cloud = cloud_pcl;

    if (cloud->width*cloud->height == 0)
    {
        object_recognition::ObjectContainer objectContainer;
        pub_detectedObject.publish(objectContainer);
        return ;
    }

    vector<std::string> fileName;
	vector<int> compression_params; //vector that stores the compression parameters of the image

	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //specify the compression technique
	//compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
	compression_params.push_back(9); //specify the compression quality

	//bool bSuccess_ = imwrite("first.jpg", img, compression_params); //write the image to file
	bool bSuccess_ = imwrite("/run/shm/object_perception/first.png", img, compression_params); //write the image to file
	
	   //pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_f (new pcl17::PointCloud<pcl17::PointXYZ>);
	   
	//std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
    //
    ROS_INFO("Get PointCloud size : %d",cloud->width*cloud->height);
	pcl17::PCDWriter writer;
	writer.write<pcl17::PointXYZ> ("/run/shm/object_perception/first.pcd", *cloud, false); 


	for (pcl17::PointCloud<pcl17::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it){
		float x = it->x;
		float y = it->y;
		float z = it->z;
		if( sqrt(x*x+y*y+z*z) < ARM_RAIDUS)
			cloud_tmp->push_back(pcl17::PointXYZ(it->x,it->y,it->z));
	}
	

	// Create the filtering object: downsample the dataset using a leaf size of 1cm

	pcl17::VoxelGrid<pcl17::PointXYZ> vg;
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_filtered (new pcl17::PointCloud<pcl17::PointXYZ>);
	vg.setInputCloud (cloud);
	//adjust?
	//vg.setLeafSize (0.001f, 0.001f, 0.001f);
	vg.setLeafSize (0.002f, 0.002f, 0.002f);
	//vg.setLeafSize (0.1f, 0.1f, 0.1f);
	vg.filter (*cloud_filtered);

//	std::cout << "PointCloud(cloud) before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
//	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

	//Create the segmentation object for the planar model and set all the parameters


//	std::cout << "PointCloud after(cloud_filtered = cloud) filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 


    if(cloud_filtered->size() != 0 )
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
	//while (cloud_filtered->points.size () > 0.1 * nr_points)
	while (cloud_filtered->points.size () > 0.3 * nr_points)
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
        //break;
	}

	// Creating the KdTree object for the search method of the extraction

    if(cloud_filtered->size() == 0){
        object_recognition::ObjectContainer objectContainer;
        pub_detectedObject.publish(objectContainer);
        return;
    }
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

	int objectNum=0;
	j = 0;


	cout << "cluster_indeces.size() = " << cluster_indices.size() << endl;

	for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{	
		float x=0,y=0,z=0;
		//float sumX=0,sumY=0,sumZ=0;
		//float pixel_x_max = 0, pixel_y_max = 0, pixel_x_min = 1280, pixel_y_min = 720;
		float pixel_x_max = 0, pixel_y_max = 0, pixel_x_min = CENTER_IMAGE_X*2, pixel_y_min = CENTER_IMAGE_Y*2;

        char pixelFileName[100];
        sprintf(pixelFileName,"/run/shm/object_perception/file_pixel%d",objectNumber);
        cout << "-----------------------------------"<< pixelFileName << endl;
        FILE* fp = fopen(pixelFileName,"w");

		pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_cluster (new pcl17::PointCloud<pcl17::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			float tmpX,tmpY;
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
			x += cloud_filtered->points[*pit].x;
			y += cloud_filtered->points[*pit].y;
			z += cloud_filtered->points[*pit].z;

			//tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CENTER_IMAGE_X;
			//tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH_Y/cloud_filtered->points[*pit].z + CENTER_IMAGE_Y;


			tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_X;
            //for webcam 1280x720
			//tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_X + (-100);
			tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH_Y/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_Y;// + 20;

			if(tmpX > pixel_x_max) pixel_x_max = tmpX;
			if(tmpY > pixel_y_max) pixel_y_max = tmpY;
			if(tmpX < pixel_x_min) pixel_x_min = tmpX;
			if(tmpY < pixel_y_min) pixel_y_min = tmpY;

            char pixelValue[100];
            sprintf(pixelValue,"(%f,%f,%f)  (%f,%f)\n",cloud_filtered->points[*pit].x,cloud_filtered->points[*pit].y,cloud_filtered->points[*pit].z,tmpX,tmpY);
            fputs (pixelValue,fp);

            //--------------------------------------------------------test--------------------------------------
//			tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CENTER_IMAGE_X_;
//			tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH_Y/cloud_filtered->points[*pit].z + CENTER_IMAGE_Y_;
//
//			if(tmpX > pixel_x_max_) pixel_x_max_ = tmpX;
//			if(tmpY > pixel_y_max_) pixel_y_max_ = tmpY;
//			if(tmpX < pixel_x_min_) pixel_x_min_ = tmpX;
//			if(tmpY < pixel_y_min_) pixel_y_min_ = tmpY;


		}



        fclose (fp);

		pixel_x_min = pixel_x_min > 0 ? pixel_x_min : 0;
		pixel_y_min = pixel_y_min > 0 ? pixel_y_min : 0;
		//pixel_x_max = pixel_x_max < 1280 ? pixel_x_max : 1280;
		pixel_x_max = pixel_x_max < CENTER_IMAGE_X*2 ? pixel_x_max : CENTER_IMAGE_X*2;
        //for webcam resolution 640x480
		//pixel_x_max = pixel_x_max < 640 ? pixel_x_max : 640;
		pixel_y_max = pixel_y_max < CENTER_IMAGE_Y*2 ? pixel_y_max : CENTER_IMAGE_Y*2;
		//pixel_y_max = pixel_y_max < 720 ? pixel_y_max : 720;




		int size = std::distance(it->indices.begin(), it->indices.end());
		x/=size;
		y/=size;
		z/=size;


//		if(z > DEPTH_LIMIT || x < PLANE_LEFT || x > PLANE_RIGHT)
//			continue;

		float objectWorldX,objectWorldY,objectWorldZ;

		if( (isObjectManipulable[j] = isObjectReachable(x,y,z,&objectWorldX,&objectWorldY,&objectWorldZ) ))
			reachableCount++;

//        objectCentroidWorld[j][0] = x;
//        objectCentroidWorld[j][1] = y;
//        objectCentroidWorld[j][2] = z;

//        objectCentroidWorld[j][0] = objectWorldX;
//        objectCentroidWorld[j][1] = objectWorldY;
//        objectCentroidWorld[j][2] = objectWorldZ;


		pixel_x = x*FOCAL_LENGTH_X/z + CALIBRATED_CENTER_IMAGE_X;// + (-0.5);
		pixel_y = y*FOCAL_LENGTH_Y/z + CALIBRATED_CENTER_IMAGE_Y;// + 20;

		center_ss << pixel_x << " " << pixel_y << " ";

		cv::Mat m = img.clone();
		//cv::Mat m_ = img_.clone();
		IplImage* iplImage = new IplImage(m);

		//IplImage* iplImage_ = new IplImage(m_);
		//IplImage* iplImage = new IplImage(m);
		//img.copyTo(iplImage);
		//iplImage = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3); 
		//iplImage = 


        //cout << "image height : " << iplImage->height << "image width : " << iplImage->width << endl;

        double topLeftX,topLeftY;
        topLeftX = max(pixel_x_min+TUNED_H_DISTANCE_TOP_LEFT,double(0.0));
        topLeftX = min(topLeftX,double(CENTER_IMAGE_X*2));
        topLeftY = max(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT,double(0.0));
        topLeftY = min(topLeftY,double(CENTER_IMAGE_Y*2));

        int width_,height_;

        //for webcam resolution 640x480
        //if(pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT <= 640 )
        if(pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT <= CENTER_IMAGE_X*2 )
            //width_ = (bottom_x+tune) - (top_x+tune) 
            //width_ = (pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT)-(pixel_x_min+TUNED_H_DISTANCE_TOP_LEFT);
            width_ = (pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT)-(topLeftX);
        else
            width_ = max((double)(CENTER_IMAGE_X*2) -topLeftX,(double)0);
            //width_ = max((double)(640) -pixel_x_min,(double)0);
        
        if(pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT <= CENTER_IMAGE_Y*2 )
        //if(pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT <= 480 )
            //height = (bottom_y+tune) - (top_y+tune)
            //height_ = (pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT)-(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT);
            height_ = (pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT)-(topLeftY);
        else
            height_ = max((double)(CENTER_IMAGE_Y*2) - topLeftY,(double)0);
            //height_ = max((double)(480) - pixel_y_min,(double)0);

        

        //int width_ = pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT < 480  ? (double)pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT : max((double)(2.0*CENTER_IMAGE_X) -pixel_x_min,(double)0);
        //int height_ = pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < 480 ? (double)pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : max((double)(480) - (TUNED_V_DISTANCE_BOTTOM_RIGHT+pixel_y_max),(double)0);
        //int height_ = pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < 480 ? (double)pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : max((double)(480) - pixel_y_min,(double)0);
        //int height_ = pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < 2.0*CENTER_IMAGE_Y ? (float)pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : max((float)(2.0*CENTER_IMAGE_Y) - (TUNED_V_DISTANCE_BOTTOM_RIGHT+pixel_y_max),(float)0);

        //cout << "value of the cropped conner without tuning" << '(' << pixel_x_min << ',' << pixel_y_min << ") (" << pixel_x_max << ',' << pixel_y_max << ')' << endl;
        //cout << "value of the cropped conner with tuning" << '(' << pixel_x_min+TUNED_H_DISTANCE_TOP_LEFT << ',' << pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT << ") (" << pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT << ',' << pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT << ')' << endl;
        cout << "value of the cropped conner with fixing and tuning" << '(' << topLeftX << ',' << topLeftY << ") (" <<topLeftX + width_<< ',' << topLeftY + height_<< ')' << endl;

        cout << "width : "<< width_<<  "height : " << height_ << endl;

        if(pixel_x_max <= 0 || pixel_x_min >= CENTER_IMAGE_X*2 || pixel_y_max <= 0 || pixel_y_min >= CENTER_IMAGE_Y*2)
            continue;

        if(width_<=0 || height_ <=0)
            continue;


        objectCentroidWorld[j][0] = objectWorldX;
        objectCentroidWorld[j][1] = objectWorldY;
        objectCentroidWorld[j][2] = objectWorldZ;

//use thie one
		//printf("topleft_x : %f, topleft_y : %f, width : %f, height : %f\n",pixel_x_min,pixel_y_min, 
        //                                                                  pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT < (float)(2.0*CENTER_IMAGE_X) ? (float)pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT : (float)(2.0*CENTER_IMAGE_X) - (TUNED_H_DISTANCE_BOTTOM_RIGHT+pixel_x_max),
        //                                                                  pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < (float)(CENTER_IMAGE_Y*2) ? (float)pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : (float)(CENTER_IMAGE_Y*2) - (TUNED_V_DISTANCE_BOTTOM_RIGHT+pixel_y_max));
        //                                                                  //pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < (float)(2.0*CENTER_IMAGE_Y) ? (float)pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : (float)(2.0*CENTER_IMAGE_Y) - (TUNED_V_DISTANCE_BOTTOM_RIGHT+pixel_y_max));
		//printf("top left : (%f,%f)\n",max(pixel_x_min-TUNED_H_DISTANCE_TOP_LEFT,(float)0),max(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT,(float)0));
		//printf("bottom right : (%f,%f)\n",min(pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_X)),min(pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_Y)));
		//ROS_INFO("write picture.%d with (%f,%f,%f,%f) iplimage->size() : %d img.size() : %d",j,pixel_x_min,pixel_y_min,pixel_x_max,pixel_y_max,iplImage->width*iplImage->height,img.rows*img.cols);

		cvSetImageROI(iplImage,cv::Rect(topLeftX,
										topLeftY,
                                        width_,
                                        height_));

		//cvSetImageROI(iplImage,cv::Rect(max(pixel_x_min+TUNED_H_DISTANCE_TOP_LEFT,(double)0),
		//								max(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT,(double)0),
        //                                width_,
        //                                height_));
        //                                //pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT < (double)(2.0*CENTER_IMAGE_X) ? pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT : (double)(2.0*CENTER_IMAGE_X) - (TUNED_H_DISTANCE_BOTTOM_RIGHT+pixel_x_max),
                                        //pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < (double)(480) ? pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : (double)(480) - (TUNED_V_DISTANCE_BOTTOM_RIGHT+pixel_y_max))); 
                                        //pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT < (float)(2.0*CENTER_IMAGE_Y) ? pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT : (float)(2.0*CENTER_IMAGE_Y) - (TUNED_V_DISTANCE_BOTTOM_RIGHT+pixel_y_max))); 

//										min(pixel_x_max-pixel_x_min+TUNED_H_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_X)),
//										min(pixel_y_max-pixel_y_min+TUNED_V_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_Y))));

//
//		cvSetImageROI(iplImage_,cv::Rect(max(pixel_x_min_-TUNED_H_DISTANCE_TOP_LEFT,(float)0),
//										max(pixel_y_min_+TUNED_V_DISTANCE_TOP_LEFT,(float)0),
//										min(pixel_x_max_-pixel_x_min_+TUNED_H_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_X)),
//										min(pixel_y_max_-pixel_y_min_+TUNED_V_DISTANCE_BOTTOM_RIGHT,(float)(2.0*CENTER_IMAGE_Y))));
//

		std::stringstream ss_,ss_2;
		std::stringstream ss_category_;

//use this one
		ss_<< "/run/shm/object_perception/picture" << objectNumber << ".png";
		ss_2<< "/run/shm/object_perception/kinect_picture_" << objectNumber << ".png";
//use this one
		bool bSuccess = imwrite(ss_.str(), cv::Mat(iplImage), compression_params); //write the image to file

		ss_category_ << "/run/shm/object_perception/" << category.c_str() << "picture" << objectNumber << ".png";
		bSuccess = imwrite(ss_category_.str(), cv::Mat(iplImage), compression_params); //write the image to file
		//bSuccess = imwrite(ss_2.str(), cv::Mat(img_), compression_params); //write the image to file



//use this one
		//------------------------------------------------------

		char comm[1000];
		//sprintf(comm,"/home/skuba/skuba_athome/object_perception/bin/extractSURF /home/skuba/skuba_athome/object_perception/picture%d.jpg /home/skuba/skuba_athome/object_perception/feature%d",j,j);
		sprintf(comm,"/home/skuba/skuba_athome/object_perception/object_recognition/bin/extractSURF /run/shm/object_perception/picture%d.png /run/shm/object_perception/feature%d",j,j);
		system(comm);

		//------------------------------------------------------


        char picturePath[100];
        char featurePath[100];
        sprintf(picturePath,"/run/shm/object_perception/picture%d.png",objectNumber);
        sprintf(featurePath,"/run/shm/object_perception/feature%d",objectNumber);
		std::stringstream sf;
		//sf << "/run/shm/object_perception/feature" << j;
		sf << "/run/shm/object_perception/picture" << j << ".png";
		//fileName.push_back(featurePath);
        
		fileName.push_back(picturePath);

		//printf("fileName = %s\n",fileName[j].c_str());
		//have to change to centroid, now it's the center of object in 2d domain
		//objectCentroidWorld[j][0] = pixel_x;
		//objectCentroidWorld[j][1] = pixel_y;
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

		std::stringstream ss;
		ss << "/run/shm/object_perception/cloud_cluster_" << objectNumber << ".pcd";
        if(cloud_cluster->size() != 0 )
            writer.write<pcl17::PointXYZ> (ss.str (), *cloud_cluster, false); 
        objectNumber++;
		//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

//		std::stringstream ss_category;
//		ss_category << "/run/shm/object_perception/" << category.c_str() << "cloud_cluster_" << j << ".pcd";

//------------------------------------cloud_cluster -------------------------
//
		//writer.write<pcl17::PointXYZ> (ss_category.str (), *cloud_cluster, false); 
		j++;
	}

	cout << "isReachable : " << reachableCount << " with ratio : " << float(reachableCount)/j << endl;

	//object_perception::Object objects[j];
	object_recognition::ObjectContainer objectContainer;

	//if((float)reachableCount/j < 0.8){
	if(false){
		objectContainer.isMove = true;
		return;
	}
	else{
		objectContainer.isMove = false;
		for(int k=0;k<objectNumber;k++){
			classifySrv.request.filepath = fileName[k];
			//verifySrv.request.objectPictureFilePath = fileName[k];
			cout << "path : " << fileName[k] << endl;

			if(classifyClient.call(classifySrv)){
			//if(verifyClient.call(verifySrv)){
			    //cout << "response from server : " << classifySrv.response.objectCategory << endl;
                cout << "response from server : category = " << classifySrv.response.objectCategory << " confidient = " << classifySrv.response.confident  << endl;
				//cout << "response from server : " << verifySrv.response.objectName << endl;

				cout << "position of centroid : " << objectCentroidWorld[k][0] << " " << objectCentroidWorld[k][1] << " " <<objectCentroidWorld[k][2] << " " <<endl;
                
				object_recognition::Object object;
				object.point.x = objectCentroidWorld[k][0];
				object.point.y = objectCentroidWorld[k][1];
				object.point.z = objectCentroidWorld[k][2];
				object.category = classifySrv.response.objectCategory;
				object.confident = classifySrv.response.confident;
				//object.category = verifySrv.response.objectName;
				object.isManipulable = isObjectManipulable[k];
				objectContainer.objects.push_back(object);
			}
			else{
				cout << "no response from server" << endl;
			}

		}
	}
	pub_detectedObject.publish(objectContainer);
	std_msgs::String string_msg;
	//string_msg.data = verification_result.str();
	//pub_verificationResult.publish(string_msg);

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

    if(enlarged_cloud->size() != 0 )
        writer.write<pcl17::PointXYZ> ("/run/shm/object_perception/centerOfObject.pcd", *enlarged_cloud, false); 

	if(maxArea == 0){
		ROS_ERROR("No object in this range.");
		return ; 
	}

}

void imageColorCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		img = cv_ptr->image;
        cv::namedWindow("window", 1);
		cv::imshow("window",img);
		cv::waitKey(3);
		//ROS_INFO("Get image size : %d",img.rows*img.cols);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}


void prepareCloud(const std_msgs::Float64 planeHeight){
    PLANE_HEIGHT = planeHeight.data;
    ready_flag = true;
}


int main (int argc, char** argv)
{
	ros::init(argc,argv,"findCenter");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	image_transport::ImageTransport it_(nh);
	image_transport::Subscriber sub_imageColor;

    //for webcam resolution 640x480
    //nh.param("FOCAL_LENGTH_X", FOCAL_LENGTH_X, 814.033512);
    //nh.param("FOCAL_LENGTH_Y", FOCAL_LENGTH_Y, 815.46674);

    //for webcam resolution 1280x720
    nh.param("FOCAL_LENGTH_X", FOCAL_LENGTH_X, 814.03512);
    nh.param("FOCAL_LENGTH_Y", FOCAL_LENGTH_Y, 815.46674);

    nh.param("CALIBRATED_CENTER_IMAGE_X", CALIBRATED_CENTER_IMAGE_X,313.73619);
    nh.param("CALIBRATED_CENTER_IMAGE_Y", CALIBRATED_CENTER_IMAGE_Y,254.26251);

    nh.param("CENTER_IMAGE_X", CENTER_IMAGE_X,320.0);
    nh.param("CENTER_IMAGE_Y", CENTER_IMAGE_Y,240.0);

    nh.param("TUNED_H_DISTANCE_TOP_LEFT", TUNED_H_DISTANCE_TOP_LEFT,0.0);
    nh.param("TUNED_V_DISTANCE_TOP_LEFT", TUNED_V_DISTANCE_TOP_LEFT,0.0);
    nh.param("TUNED_H_DISTANCE_BOTTOM_RIGHT", TUNED_H_DISTANCE_BOTTOM_RIGHT,0.0);
    nh.param("TUNED_V_DISTANCE_BOTTOM_RIGHT", TUNED_V_DISTANCE_BOTTOM_RIGHT,0.0);
    nh.param("ARM_RAIDUS", ARM_RAIDUS,1.1);

    cout << "FOCAL_LENGTH_X " <<  FOCAL_LENGTH_X << " FOCAL_LENGTH_Y " << FOCAL_LENGTH_Y << endl;
    cout << "CALIBRATED_CENTER_IMAGE_X " <<  CALIBRATED_CENTER_IMAGE_X << " CALIBRATED_CENTER_IMAGE_Y " << CALIBRATED_CENTER_IMAGE_Y << endl;
    cout << "CENTER_IMAGE_X " <<  CENTER_IMAGE_X << " CENTER_IMAGE_Y " << CENTER_IMAGE_Y << endl;
    cout << "TUNED_H_DISTANCE_TOP_LEFT " <<  TUNED_H_DISTANCE_TOP_LEFT << " TUNED_V_DISTANCE_TOP_LEFT " << TUNED_V_DISTANCE_TOP_LEFT << endl;
    cout << "TUNED_H_DISTANCE_BOTTOM_RIGHT " <<  TUNED_H_DISTANCE_BOTTOM_RIGHT << " TUNED_V_DISTANCE_BOTTOM_RIGHT" << TUNED_V_DISTANCE_BOTTOM_RIGHT << endl;
    cout << "ARM_RAIDUS " << ARM_RAIDUS << endl;

    //char mkdirCommand[1000];
    //sprintf(comm,"/home/skuba/skuba_athome/object_perception/bin/extractSURF /home/skuba/skuba_athome/object_perception/picture%d.jpg /home/skuba/skuba_athome/object_perception/feature%d",j,j);
    //sprintf(mkdirCommand,"/home/skuba/skuba_athome/object_perception/object_recognition/bin/extractSURF /run/shm/object_perception/picture%d.png /run/shm/object_perception/feature%d",j,j);
    system("mkdir /run/shm/object_perception");


	pubObjectNum = n.advertise<std_msgs::String>("object_number", 1000);
	center_pub = n.advertise<std_msgs::String>("center_pcl_object", 1000);
	pub_detectedObject = n.advertise<object_recognition::ObjectContainer>("/detected_object", 1000);
//	ros::Subscriber sub = n.subscribe("/camera/depth_registered/points",1,depthCB);
	//ros::Subscriber sub = n.subscribe("/camera/depth/points",1,depthCB);
	ros::Subscriber sub = n.subscribe("/cloud_tf",1,depthCB);
	ros::Subscriber sub_ = n.subscribe("/search_object",1, prepareCloud);
	sub_imageColor = it_.subscribe("/logitech_cam/image_raw", 1, imageColorCb);

	
	verifyClient = n.serviceClient<object_recognition::verifyObject>("verifyObject");
	//object_recognition::verifyObject verifySrv;

	//classifyClient = n.serviceClient<object_recognition::classifyObject>("classifyObject");
	classifyClient = n.serviceClient<object_recognition::classifyObject>("/object_recognition/verify_object");

    //verify_object = rospy.ServiceProxy('/object_recognition/verify_object',classifyObject)
	//object_recognition::classifyObject classifySrv;

	isManipulableClient = n.serviceClient<manipulator::isManipulable>("isManipulable");
	manipulator::isManipulable isManipulatableSrv;

	listener = new tf::TransformListener();
	ros::spin();

	return (0);
}
