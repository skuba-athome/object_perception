#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/transforms.h>
#include <cv.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>

#include <shape_tools/solid_primitive_dims.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include <math.h> 
#include <string>
#include <sstream>
#include <algorithm>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tabletop/Table.h"
#include "tabletop/TabletopObjectDetection.h"
#include "tabletop/marker_generator.h"
#include "tabletop/TabletopSegmentation.h"


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
  ros::ServiceServer object_dectector_srv_;
  ros::ServiceClient table_seg_srv_;
  

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  std::string output_dir;  

  tf::TransformListener* listener;

  double FOCAL_LENGTH_X;
  double FOCAL_LENGTH_Y;
  double RESOLUTION_WIDTH;
  double RESOLUTION_HEIGHT;
  
  double CALIBRATED_CENTER_IMAGE_X_;
  double CALIBRATED_CENTER_IMAGE_Y_;
  double TUNED_H_DISTANCE_TOP_LEFT_;
  double TUNED_V_DISTANCE_TOP_LEFT_;
  double TUNED_H_DISTANCE_BOTTOM_RIGHT_;
  double TUNED_V_DISTANCE_BOTTOM_RIGHT_;

  // callback 
  bool serviceCallback(TabletopObjectDetection::Request &request, TabletopObjectDetection::Response &response);

  //! Performs object detection on the given clusters, can also merge clusters based on detection result
  template <class PointCloudType>
  void objectDetection(std::vector<PointCloudType> clusters, const tabletop::Table table,
	std::vector<geometry_msgs::Point> &centriods, std::vector<shape_msgs::SolidPrimitive> &solid_boxes);

  void clearOldMarkers(std::string frame_id);

  cv::Mat object_image;
  ros::NodeHandle image_nh_;
  image_transport::ImageTransport transport_;   
  image_transport::Subscriber image_sub_;  
  //sensor_msgs::Image::ConstPtr image;
  std::string camera_topic;
  
public:

  //! Subscribes to and advertises topics; initializes fitter
  TabletopObjectDetector(ros::NodeHandle nh);

  //! Empty stub
  ~TabletopObjectDetector() {}

};


TabletopObjectDetector::TabletopObjectDetector(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), transport_(nh)
{
  ROS_INFO("TabletopObjectDetector ...");
  listener = new tf::TransformListener();

  std::string service_name;
  priv_nh_.param<std::string>("segmentation_srv", service_name, "/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", service_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  table_seg_srv_ = nh_.serviceClient<TabletopSegmentation>(service_name, true);

  num_markers_published_ = 1;
  current_marker_id_ = 1;
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);
  ROS_INFO("  marker publish");

  //initialize operational flags
  priv_nh_.param<double>("FOCAL_LENGTH_X", FOCAL_LENGTH_X, 981.383623674383);
  priv_nh_.param<double>("FOCAL_LENGTH_Y", FOCAL_LENGTH_Y, 978.403976366632);
  priv_nh_.param<double>("RESOLUTION_WIDTH", RESOLUTION_WIDTH, 1280);
  priv_nh_.param<double>("RESOLUTION_HEIGHT", RESOLUTION_HEIGHT, 720);

  priv_nh_.param<double>("CALIBRATED_CENTER_IMAGE_X_", CALIBRATED_CENTER_IMAGE_X_, 640);
  priv_nh_.param<double>("CALIBRATED_CENTER_IMAGE_Y_", CALIBRATED_CENTER_IMAGE_Y_, 360);
  priv_nh_.param<double>("TUNED_H_DISTANCE_TOP_LEFT_", TUNED_H_DISTANCE_TOP_LEFT_, 0);
  priv_nh_.param<double>("TUNED_V_DISTANCE_TOP_LEFT_", TUNED_V_DISTANCE_TOP_LEFT_, 0);
  priv_nh_.param<double>("TUNED_H_DISTANCE_BOTTOM_RIGHT_", TUNED_H_DISTANCE_BOTTOM_RIGHT_, 0);
  priv_nh_.param<double>("TUNED_V_DISTANCE_BOTTOM_RIGHT_", TUNED_V_DISTANCE_BOTTOM_RIGHT_, 0);
  
  object_dectector_srv_ = nh_.advertiseService("/tabletop_object_detection", &TabletopObjectDetector::serviceCallback, this);
  ROS_INFO("Tabletop Object Dectection node ready");
  
}

bool TabletopObjectDetector::serviceCallback(TabletopObjectDetection::Request &request, TabletopObjectDetection::Response &response)
{
  // service callback
  tabletop::TabletopSegmentation segmentation_srv;
  if (!table_seg_srv_.call(segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    response.result = segmentation_srv.response.OTHER_ERROR;
    return true;
  }
  response.result = segmentation_srv.response.result;
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
  //tf::Transform table_trans;
  //tf::poseMsgToTF(table.pose.pose, table_trans);
  //tf::StampedTransform table_trans_stamped(table_trans, table.pose.header.stamp, table.pose.header.frame_id, "table_frame");
  
  camera_topic = "/external_cam/image_raw"; 
  sensor_msgs::Image::ConstPtr image = ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, nh_, ros::Duration(3.0));  
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);      
  object_image = cv_ptr->image;          
  ROS_INFO("TabletopObjectDetector get image size : %d %d", object_image.rows, object_image.cols);    
    
  for (size_t i = 0; i < clusters.size (); ++i)
  {
    ROS_INFO("start transform");
    if (clusters[i].header.frame_id != table.pose.header.frame_id)
    {
      ROS_ERROR("Recognition node requires all clusters to be in the same frame as the table");
      return false;
    }
    clusters[i].header.stamp = table.pose.header.stamp;
    
    listener->waitForTransform("external_cam", table.pose.header.frame_id, table.pose.header.stamp, ros::Duration(3.0));         
    try
    {
      ROS_INFO_STREAM("Before transform clusters[i].z " << clusters[i].points[0].x << " " << clusters[i].points[0].z);
      ROS_INFO_STREAM("   Header " << clusters[i].header.frame_id);
      //listener.transformPointCloud("table_frame", clusters[i], clusters[i]); 
      sensor_msgs::PointCloud2 clusters_pc2;
      sensor_msgs::convertPointCloudToPointCloud2 (clusters[i], clusters_pc2);
      pcl_ros::transformPointCloud("external_cam", clusters_pc2, clusters_pc2, *listener); 
      sensor_msgs::convertPointCloud2ToPointCloud (clusters_pc2, clusters[i]);
      ROS_INFO_STREAM("   Header Cluster " << clusters[i].header.frame_id);

      ROS_INFO_STREAM("After transform clusters[i].z " << clusters[i].points[0].x << " " << clusters[i].points[0].z);
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
  response.table = table;
  response.clusters = clusters;
  response.centriods =  centriods;
  response.solid_boxes = solid_boxes; 

  ROS_INFO_STREAM("objectDetectionResults publish " << response.result << " " 
  	<< response.centriods[0].x << " " << response.centriods[0].y << " " << response.centriods[0].z);

  clearOldMarkers(table.pose.header.frame_id);

  return true;
} 

void TabletopObjectDetector::clearOldMarkers(std::string frame_id)
{
  ROS_INFO_STREAM("delete current marker id " << current_marker_id_);
  for (int id=current_marker_id_; id < num_markers_published_; id++)
  {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.header.frame_id = frame_id;
    delete_marker.id = id;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    delete_marker.ns = "tabletop_node";
    marker_pub_.publish(delete_marker);
    ROS_INFO_STREAM("delete marker id " << id);
  }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;

}

template <class PointCloudType>
void TabletopObjectDetector::objectDetection(std::vector<PointCloudType> clusters, const tabletop::Table table,
	std::vector<geometry_msgs::Point> &centriods, std::vector<shape_msgs::SolidPrimitive> &solid_boxes)
{
	// process for each cluster PointCloudType
	// all computed in table frame
	for (size_t i = 0; i < clusters.size (); ++i)
	{
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

    // create marker for object
    visualization_msgs::Marker objectMarker = 
		MarkerGenerator::createMarker(table.pose.header.frame_id, 0, 
			fabs(max_points.x - min_points.x), fabs(max_points.y - min_points.y), fabs(max_points.z - min_points.z), 
			1, 0, 0, visualization_msgs::Marker::CUBE, current_marker_id_++, "tabletop_node", table.pose.pose);
    objectMarker.pose.position.x = table.x_min + fabs(max_points.x - min_points.x)/2.0;
		objectMarker.pose.position.z = table.pose.pose.position.z + 0.1 - 0.02;
		marker_pub_.publish(objectMarker);
		ROS_INFO("Object marker publish");

		centriods.push_back(objectMarker.pose.position);
  	ROS_INFO("Compute centriods completed x y z: %.2f %.2f %.2f", centriods[i].x, centriods[i].y, centriods[i].z);


		// crop image object and save as .png
		// transform webcam to table_frame
		// resolution 1280x720
    output_dir = ros::package::getPath("tabletop") + "/out";
    
    float pixel_x_max = 0, pixel_y_max = 0, pixel_x_min = 1280, pixel_y_min = 720;
    float tmp_x = 0, tmp_y =0;
    float x_avg=0, y_avg=0, z_avg=0;
    float pixel_x = 0, pixel_y = 0;
    int width_=0, height_=0;
    double topLeftX, topLeftY;
    
    // picture file name
    std::stringstream picFileName;
    picFileName << output_dir << "/pixel" << i;
    FILE* fp = fopen(picFileName.str().c_str(),"w");
    for (unsigned int j=0; j<clusters[i].points.size(); ++j) 
  	{
      x_avg += clusters[i].points[j].x;
      y_avg += clusters[i].points[j].y;
      z_avg += clusters[i].points[j].z;

  		tmp_x = clusters[i].points[j].x * FOCAL_LENGTH_X/clusters[i].points[j].z + CALIBRATED_CENTER_IMAGE_X_; 
  		tmp_y = clusters[i].points[j].y * FOCAL_LENGTH_Y/clusters[i].points[j].z + CALIBRATED_CENTER_IMAGE_Y_; 

      if(tmp_x > pixel_x_max) pixel_x_max = tmp_x;
      if(tmp_y > pixel_y_max) pixel_y_max = tmp_y;
      if(tmp_x < pixel_x_min) pixel_x_min = tmp_x;
      if(tmp_y < pixel_y_min) pixel_y_min = tmp_y;            

      char pixelValue[100];
      sprintf(pixelValue,"(%f,%f,%f)  (%f,%f)\n",clusters[i].points[j].x, clusters[i].points[j].y, clusters[i].points[j].z, tmp_x, tmp_y);
      fputs (pixelValue,fp);
    }   
    fclose (fp);
    
    // save clusters to pointcloud file
    pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2 (clusters[i], pc2);    
    pcl::fromROSMsg(pc2, cloud_cluster); 
    std::stringstream pcdFileName;
    pcdFileName << output_dir << "/cluster" << i << ".pcd";        
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (pcdFileName.str(), cloud_cluster, false);
    ROS_INFO_STREAM("Saved object pointcloud file " << i);

    ROS_INFO("\n\npixel(x_min,x_max,y_min,y_max) = (%f %f %f %f)", pixel_x_min, pixel_x_max, pixel_y_min, pixel_y_max);
    // picture size and mapping position
    pixel_x_min = pixel_x_min > 0 ? pixel_x_min : 0;
    pixel_y_min = pixel_y_min > 0 ? pixel_y_min : 0;
    pixel_x_max = pixel_x_max < RESOLUTION_WIDTH ? pixel_x_max : RESOLUTION_WIDTH;
    pixel_y_max = pixel_y_max < RESOLUTION_HEIGHT ? pixel_y_max : RESOLUTION_HEIGHT;        

    x_avg /= clusters[i].points.size();
    y_avg /= clusters[i].points.size();
    z_avg /= clusters[i].points.size();

    pixel_x = x_avg*FOCAL_LENGTH_X/z_avg + CALIBRATED_CENTER_IMAGE_X_;
    pixel_y = y_avg*FOCAL_LENGTH_Y/z_avg + CALIBRATED_CENTER_IMAGE_Y_;
    
    topLeftX = std::max(pixel_x_min + TUNED_H_DISTANCE_TOP_LEFT_, double(0.0) );
    topLeftX = std::min(topLeftX, double(RESOLUTION_WIDTH) );
    topLeftY = std::max(pixel_y_min + TUNED_V_DISTANCE_TOP_LEFT_, double(0.0) );
    topLeftY = std::min(topLeftY, double(RESOLUTION_HEIGHT) );

    if(pixel_x_max + TUNED_H_DISTANCE_BOTTOM_RIGHT_ <= RESOLUTION_WIDTH )
    {
      width_ = (pixel_x_max + TUNED_H_DISTANCE_BOTTOM_RIGHT_)-(topLeftX);
      ROS_INFO("WIDTH %d", width_);
    }
    else
        width_ = std::max((double)(RESOLUTION_WIDTH) - topLeftX, (double)0);            

    if(pixel_y_max + TUNED_V_DISTANCE_BOTTOM_RIGHT_ <= RESOLUTION_HEIGHT )
    {
      height_ = (pixel_y_max + TUNED_V_DISTANCE_BOTTOM_RIGHT_)-(topLeftY);
      ROS_INFO("HEIGHT %d", height_);
    }
    else
        height_ = std::max((double)(RESOLUTION_HEIGHT) - topLeftY, (double)0);

    if(pixel_x_max <= 0 || pixel_x_min >= RESOLUTION_WIDTH || pixel_y_max <= 0 || pixel_y_min >= RESOLUTION_HEIGHT)
        continue;
    if(width_<=0 || height_ <=0)
        continue;

    cv::Mat m = object_image.clone();
    ROS_INFO("Get image size : %d %d", object_image.rows, object_image.cols);
    
    IplImage* iplImage = new IplImage(m);
    cvSetImageROI(iplImage, cv::Rect(topLeftX, topLeftY, fabs(width_), fabs(height_)) );

    //write the image to file
    std::vector<int> compression_params; //vector that stores the compression parameters of the image
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); 
    compression_params.push_back(9); //specify the compression quality
    
    std::stringstream objectFileName;        
    objectFileName << output_dir << "/object" << i << ".png";
    bool bSuccess = imwrite(objectFileName.str(), cv::Mat(iplImage), compression_params);
    
    std::vector<std::string> fileName;
    fileName.push_back(objectFileName.str());
    ROS_INFO_STREAM("Saved object picture No " << i << " SUCCESS " << bSuccess );

    ROS_INFO("pixel(x_min,x_max,y_min,y_max) = (%f %f %f %f)", pixel_x_min, pixel_x_max, pixel_y_min, pixel_y_max);
    ROS_INFO("tmp(x,y) = (%f %f)", tmp_x, tmp_y);
    ROS_INFO("avg(x,y,z) = (%f %f %f)", x_avg, y_avg, z_avg);
    ROS_INFO("pixel(x,y) = (%f %f)", pixel_x, pixel_y);
    ROS_INFO("topLeft(x,y) = (%f %f)", topLeftX, topLeftY);
    ROS_INFO("width_ height_ = (%d %d)", width_, height_);

    cv::namedWindow("window", 1);
    cv::imshow("window",cv::Mat(iplImage));
    cv::waitKey(3);

	}
}

} // namespace tabletop

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "tabletop_object_detection");
	ros::NodeHandle nh;
  tabletop::TabletopObjectDetector node(nh);
  ros::spin();
	return 0;
}
