// read clusters from tabletop_segmentation service
// find centriods

// publish solid shapes and collision objects
// publish markers

// crop object images and save to .png in order to recognize

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <math.h> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


#include "tabletop/marker_generator.h"
#include "tabletop/TabletopSegmentation.h"
#include <tabletop/Table.h>
#include <tabletop/ObjectDetection.h>
#include <shape_tools/solid_primitive_dims.h>

namespace tabletop {
class TabletopObjectDetector
{
private:
  //! The node handle
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  //! Listener for incoming new-style point clouds
  ros::Subscriber cloud_new_sub_;
  //! Publisher for markers
  ros::Publisher marker_pub_;

  ros::ServiceClient table_seg_srv_;

  ros::Publisher object_dectector_pub_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;


  //! A tf transform listener
  tf::TransformListener listener_;
  
  bool serviceCallback();

  //! Performs object detection on the given clusters, can also merge clusters based on detection result
  template <class PointCloudType>
  void objectDetection(std::vector<PointCloudType> clusters, const tabletop::Table table,
	std::vector<geometry_msgs::Point> &centriods, std::vector<shape_msgs::SolidPrimitive> &solid_boxes);
        
  //void clearOldMarkers(std::string frame_id);

public:
  //! Subscribes to and advertises topics; initializes fitter
  TabletopObjectDetector(ros::NodeHandle nh);

  //! Empty stub
  ~TabletopObjectDetector() {}


};


TabletopObjectDetector::TabletopObjectDetector(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
  num_markers_published_ = 1;
  current_marker_id_ = 1;
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

  std::string service_name;
  priv_nh_.param<std::string>("segmentation_srv", service_name, "/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", service_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  table_seg_srv_ = nh_.serviceClient<TabletopSegmentation>(service_name, true);

  object_dectector_pub_ = nh_.advertise<tabletop::ObjectDetection>("object_detection", 10);
  bool callbackSuccess = TabletopObjectDetector::serviceCallback();

  
  ROS_INFO("Tabletop Object Dectection node ready");
}


bool TabletopObjectDetector::serviceCallback()
{
  
  tabletop::ObjectDetection objectDetectionResults;

  // service callback
  tabletop::TabletopSegmentation segmentation_srv;
  if (!table_seg_srv_.call(segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    objectDetectionResults.result = segmentation_srv.response.OTHER_ERROR;
    return true;
  }
  objectDetectionResults.result = segmentation_srv.response.result;
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    return true;
  }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());

  tabletop::Table table = segmentation_srv.response.table;
 
  std::vector<sensor_msgs::PointCloud> clusters = segmentation_srv.response.clusters;
  if (clusters.empty() ) return true;

  //convert clusters point clouds to table frame
  ROS_INFO("Transform clusters point clouds to table frame");
  tf::Transform table_trans;
  tf::poseMsgToTF(table.pose.pose, table_trans);
  tf::StampedTransform table_trans_stamped(table_trans, table.pose.header.stamp, 
                                           table.pose.header.frame_id, "table_frame");
  tf::TransformListener listener;
  listener.setTransform(table_trans_stamped);
  for (size_t i = 0; i < clusters.size (); ++i)
  {
    if (clusters[i].header.frame_id != table.pose.header.frame_id)
    {
      ROS_ERROR("Recognition node requires all clusters to be in the same frame as the table");
      return false;
    }
    clusters[i].header.stamp = table.pose.header.stamp;
    try
    {
      listener.transformPointCloud("table_frame", clusters[i], clusters[i]); 
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Recognition node failed to transform cluster from frame %s into table frame; Exception: %s", 
                clusters[i].header.frame_id.c_str(), ex.what());
      return false;
    }
  }
  ROS_DEBUG("Clusters converted to table frame");
  

  std::vector<geometry_msgs::Point> centriods;
  std::vector<shape_msgs::SolidPrimitive> solid_boxes;

  //run detection
  objectDetection<sensor_msgs::PointCloud> (clusters, table, centriods, solid_boxes);

  // publish object detector 
  objectDetectionResults.table = table;
  objectDetectionResults.clusters = clusters;
  objectDetectionResults.centriods =  centriods;
  objectDetectionResults.solid_boxes = solid_boxes;
  object_dectector_pub_.publish(objectDetectionResults);
  ROS_INFO_STREAM("objectDetectionResults publish " << objectDetectionResults.result << " " 
  	<< centriods[0].x << " " << centriods[0].y << " " << centriods[0].z);

  //publishFitMarkers(response.models, request.table);
  ///clearOldMarkers(request.table.pose.header.frame_id);


  return true;
} 


template <class PointCloudType>
void TabletopObjectDetector::objectDetection(std::vector<PointCloudType> clusters, const tabletop::Table table,
	std::vector<geometry_msgs::Point> &centriods, std::vector<shape_msgs::SolidPrimitive> &solid_boxes)
{
	// process for each cluster PointCloudType
	// all computed in table frame
	
	for (size_t i = 0; i < clusters.size (); ++i)
	{
		// compute a centriod 
		geometry_msgs::Point center;
  		center.x = center.y = center.z = 0;	    
  		for (unsigned int j=0; j<clusters[i].points.size(); ++j) 
  		{
    		center.x += clusters[i].points[j].x;
    		center.y += clusters[i].points[j].y;
    		center.z += clusters[i].points[j].z;
  		}
  		center.x /= clusters[i].points.size();
  		center.y /= clusters[i].points.size();
  		center.z /= clusters[i].points.size();
  		
  		centriods.push_back(center);
  		ROS_INFO("Compute centriods completed x y z: %.2f %.2f %.2f", center.x, center.y, center.z);
	
		// create solid boxes to cover clusters 
		//sensor_msgs::PointCloud2 clusters_pointcloud2;
		//pcl::PointCloud<pcl::PointXYZRGB >::Ptr clusters_pcl (new pcl::PointCloud<pcl::PointXYZRGB>); 
		//Eigen::Vector4f min_points, max_points;
		//sensor_msgs::convertPointCloudToPointCloud2 (clusters[i], clusters_pointcloud2);
		//pcl::fromROSMsg(clusters_pointcloud2, *clusters_pcl);
		//pcl::getMinMax3D(*clusters_pcl, min_points, max_points);

		geometry_msgs::Point min_points, max_points;
  		min_points.x = clusters[i].points[0].x;
  		min_points.y = clusters[i].points[0].y;
  		min_points.z = clusters[i].points[0].z;
  		max_points.x = clusters[i].points[0].x;
  		max_points.y = clusters[i].points[0].y;
  		max_points.z = clusters[i].points[0].z;

  		for (unsigned int j=0; j<clusters[i].points.size(); ++j) 
  		{
  			if (clusters[i].points[j].x < min_points.x) min_points.x = clusters[i].points[j].x;
  			if (clusters[i].points[j].y < min_points.y) min_points.y = clusters[i].points[j].y;
  			if (clusters[i].points[j].z < min_points.z) min_points.z = clusters[i].points[j].z;
  			
  			if (clusters[i].points[j].x > max_points.x) max_points.x = clusters[i].points[j].x;
  			if (clusters[i].points[j].y > max_points.y) max_points.y = clusters[i].points[j].y;
  			if (clusters[i].points[j].z > max_points.z) max_points.z = clusters[i].points[j].z;
  		}

		
    	shape_msgs::SolidPrimitive solid_box;
    	solid_box.type = shape_msgs::SolidPrimitive::BOX;
    	solid_box.dimensions.resize(3);
    	solid_box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = fabs(max_points.x - min_points.x);
    	solid_box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = fabs(max_points.y - min_points.y);
    	solid_box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = fabs(max_points.z - min_points.z);

    	solid_boxes.push_back(solid_box);
    	ROS_INFO("Create object solid boxes completed size x y z: %.2f %.2f %.2f", 
    		fabs(max_points.x - min_points.x), fabs(max_points.y - min_points.y), fabs(max_points.z - min_points.z));


    	visualization_msgs::Marker originMarker = 
		MarkerGenerator::createMarker(table.pose.header.frame_id, 0, 0.1, 0.1, 0.02, 0, 1, 1, 
			visualization_msgs::Marker::CUBE, current_marker_id_++, "tabletop_node", table.pose.pose);
		originMarker.pose.position.y = table.pose.pose.position.y - 0.5;
		originMarker.pose.position.z = table.pose.pose.position.z*2.0 - 0.02;

		ROS_INFO_STREAM("originMarker " << originMarker.pose.position.x << " "
			<<originMarker.pose.position.y << " "
			<<originMarker.pose.position.z << " "
			<<originMarker.pose.orientation.x << " "
			<<originMarker.pose.orientation.y << " "
			<<originMarker.pose.orientation.z << " "
			<<originMarker.pose.orientation.w << " "
			<< table.pose.header
			);
		marker_pub_.publish(originMarker);

    	// create marker for objecT    	
    	visualization_msgs::Marker objectMarker = 
		MarkerGenerator::createMarker(table.pose.header.frame_id, 0, 
			fabs(max_points.x - min_points.x), fabs(max_points.y - min_points.y), fabs(max_points.z - min_points.z), 
			1, 0, 0, 
			visualization_msgs::Marker::CUBE, current_marker_id_++, "tabletop_node", table.pose.pose);
		objectMarker.pose.position.y = table.pose.pose.position.y - 0.5;
		objectMarker.pose.position.z = table.pose.pose.position.z + 0.4 - fabs(max_points.z - min_points.z);
		marker_pub_.publish(objectMarker);
		ROS_INFO("Object marker publish");


		// crop image object and save as .png
		// transform webcam to table_frame

		// resolution 1280x720
		/*
        float pixel_x_max = 0, pixel_y_max = 0, pixel_x_min = 1280, pixel_y_min = 720;
        float x = 0, y =0;
        // picture file name
        string output_dir;
        stringstream picFileName;
        pixelFileName << output_dir << "cluster" << i;
        FILE* fp = fopen(picFileName.str().c_str(),"w");

        for (unsigned int j=0; j<clusters[i].points.size(); ++j) 
  		{
    		x = clusters[i].points[j].x * FOCAL_LENGTH_X/clusters[i].points[j].z;
    		y = clusters[i].points[j].y * FOCAL_LENGTH_Y/clusters[i].points[j].z;
    		center.z += clusters[i].points[j].z;
  		}

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {

            float tmpX,tmpY;
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 

            x += cloud_filtered->points[*pit].x;
            y += cloud_filtered->points[*pit].y;
            z += cloud_filtered->points[*pit].z;

            //for webcam 1280x720
			float FOCAL_LENGTH_X = 981.383623674383;
  			float FOCAL_LENGTH_Y = 978.403976366632;
           	tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_X;
            tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH_Y/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_Y;// + 20;

            if(tmpX > pixel_x_max) pixel_x_max = tmpX;
            if(tmpY > pixel_y_max) pixel_y_max = tmpY;
            if(tmpX < pixel_x_min) pixel_x_min = tmpX;
            if(tmpY < pixel_y_min) pixel_y_min = tmpY;
            

            char pixelValue[100];
            sprintf(pixelValue,"(%f,%f,%f)  (%f,%f)\n",cloud_filtered->points[*pit].x,cloud_filtered->points[*pit].y,cloud_filtered->points[*pit].z,tmpX,tmpY);
            fputs (pixelValue,fp);
        }
		*/	
	}
	
}
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "tabletop_object_detection");
	ros::NodeHandle nh;

	tabletop::TabletopObjectDetector node(nh);
	ros::spin();
	return 0;
}
