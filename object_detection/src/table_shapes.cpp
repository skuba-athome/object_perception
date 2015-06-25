#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <object_recognition_msgs/TableArray.h>
//#include <shape_msgs/SolidPrimitive.h>
#include <object_detection/ObjectDetection.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/CollisionObject.h>
#include <pcl_ros/transforms.h>

class TableShape
{
public:
  TableShape()
  {
    table_sub = nh.subscribe("table_array", 1, &TableShape::tableCallback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("table_marker", 1);
    solid_shape_pub = nh.advertise<object_detection::ObjectDetection>("table_shape", 1);
    pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    listener = new tf::TransformListener();
    //solid_shape_pub = nh.advertise<shape_msgs::SolidPrimitive>("table_shape", 1);
    //solid_shape_serv = nh.advertiseService("table_shape", &TableShape::getSolidPrimitive, this);
  }

  void tableCallback(const object_recognition_msgs::TableArrayPtr& table_array)
  {
    //ROS_INFO("tableCallback");
     if( (int)table_array->tables.size() == 0)
    { 
      ROS_INFO("Table not found");
      return;
    }

    for (size_t index = 0; index < 1; ++index) 
    //for (std::vector<int>::iterator it = table_array.begin(); table_array.end(); it++)
    {
        //object_recognition_msgs::Table table = table_array->tables[index];
        table = table_array->tables[index];

        std_msgs::Header table_header = table.header;
        //geometry_msgs::Pose table_pose = table.pose;
        table_pose = table.pose;
        //ROS_INFO("POSE: [%f %f %f]", table_pose.position.x, table_pose.position.y, table_pose.position.z);
        
        std::vector<geometry_msgs::Point> table_convex_hull = table.convex_hull;
        //ROS_INFO("  table_convex_hull %d",(int)table_convex_hull.size());

        double x_max;
        double x_min;
        double y_max;
        double y_min;
        double z = 0;
        

		if (!table_convex_hull.empty()) 
		{
			x_min = table_convex_hull[0].x;
			x_max = table_convex_hull[0].x;
			y_min = table_convex_hull[0].y;
			y_max = table_convex_hull[0].y;
		}  
		
		for (size_t i=1; i<table_convex_hull.size(); ++i) 
		{
			if (table_convex_hull[i].x < x_min && table_convex_hull[i].x >-3.0) 
				x_min = table_convex_hull[i].x ;
			if (table_convex_hull[i].x > x_max && table_convex_hull[i].x < 3.0) 
				x_max = table_convex_hull[i].x ;
			if (table_convex_hull[i].y < y_min && table_convex_hull[i].y >-3.0) 
				y_min = table_convex_hull[i].y ;
			if (table_convex_hull[i].y > y_max && table_convex_hull[i].y < 3.0) 
				y_max = table_convex_hull[i].y ;
		}
		size_x = x_max-x_min;
        size_y = y_max-y_min;

		//compute centroid
		centroid.x = centroid.y = centroid.z = 0.0;
		for (size_t i=0; i<table_convex_hull.size(); i++)
		{
		centroid.x += table_convex_hull[i].x;
		centroid.y += table_convex_hull[i].y;
		centroid.z += table_convex_hull[i].z;
		}
		centroid.x /= table_convex_hull.size();
		centroid.y /= table_convex_hull.size();
		centroid.z /= table_convex_hull.size();

        //getCloudMarker(table);
        getCloudMarker(centroid, table_pose, size_x, size_y);
        publishObjectArray(table);
        getSolidPrimitive();
    }
  }

  //void getCloudMarker(const object_recognition_msgs::Table table)
  void getCloudMarker(const geometry_msgs::Point centriod ,const geometry_msgs::Pose table_pose, const double size_x, const double size_y)
  {
      //ROS_INFO("getCloudMarker");
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/camera_depth_optical_frame";
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.stamp = ros::Time();
      //marker.lifetimer = ros::Duration(5);
      
      marker.type = visualization_msgs::Marker::CUBE;

      //marker.pose.position.x = table_pose.position.x;
      //marker.pose.position.y = table_pose.position.y;
      //marker.pose.position.z = table_pose.position.z;
      
      marker.pose.position.x = centroid.x;
      marker.pose.position.y = centroid.y;
      marker.pose.position.z = table_pose.position.z;

      marker.pose.orientation.x = table_pose.orientation.x;
      marker.pose.orientation.y = table_pose.orientation.y;
      marker.pose.orientation.z = table_pose.orientation.z;
      marker.pose.orientation.w = table_pose.orientation.w;
      
      marker.scale.x = size_x;
      marker.scale.y = size_y;
      marker.scale.z = 0.03;

      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      
      ROS_INFO("table_centriod %f %f %f %f %f", table_pose.position.x, table_pose.position.y, table_pose.position.z, centriod.x, centriod.y );
      //ROS_INFO("centriod %f %f %f", centriod.x, centriod.y, centriod.z );

      /*
      marker.type = visualization_msgs::Marker::POINTS;
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = 0.1;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      for (size_t i = 0; i < table.convex_hull.size(); i++) 
      {
          geometry_msgs::Point point;
          point.x = table.convex_hull[i].x;
          point.y = table.convex_hull[i].y;
          point.z = table.convex_hull[i].z;
          marker.points.push_back(point);
      }
      //ROS_INFO("table.convex_hull %d",(int)table.convex_hull.size());
      */
      marker_pub.publish(marker);
      //return marker;
  }

  //bool getSolidPrimitive(using_markers::TableShape::Request &req, using_markers::TableShape::Response &res)
  void getSolidPrimitive()
  {

    shape_msgs::SolidPrimitive solid_shape;
    solid_shape.type = shape_msgs::SolidPrimitive::BOX;
    solid_shape.dimensions.resize(3);
    //solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
    //solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.3;
    //solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size_x;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size_y;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.01;
    
    //solid_shape.dimensions.push_back(0.3);
    //solid_shape.dimensions.push_back(0.3);
    //solid_shape.dimensions.push_back(0.3);
    //res.solid_shape = solid_shape;
    //res.centriod = table_pose;
    
    //object_detection::ObjectDetection msg;
    msg.solid_shape = solid_shape;
    //msg.centriod = table_pose.position;
    
    //solid_shape_pub.publish(solid_shape);

    // publish collision message to moveit
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";

    // remove table
    co.id = "table";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add table
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = solid_shape.type;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = msg.solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = msg.solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = msg.solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = msg.centriod.x;
    co.primitive_poses[0].position.y = msg.centriod.y;
    co.primitive_poses[0].position.z = msg.centriod.z;

    solid_shape_pub.publish(msg);
    pub_co.publish(co);


    //ROS_INFO("Table Plane Primitive Shape: [%d %.2f %.2f]", (int)res.solid_shape.type, size_x, size_y);
    //return true;
  }


  Eigen::Matrix4f getHomogeneousMatrix(std::string input_frame,std::string des_frame)
  {

    tf::StampedTransform transform;
    try{
      listener->lookupTransform(des_frame, input_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    } 
    Eigen::Matrix4f T;
    pcl_ros::transformAsMatrix(transform,T);
    return T;
  }

  void publishObjectArray(object_recognition_msgs::Table table) //std::vector<person> &tracklist
  {
    std::string camera_optical_frame = "camera_rgb_optical_frame";
    std::string robot_frame = "base_link";

    std_msgs::Header table_header = table.header;
    //people_detection::PersonObjectArray pubmsg;
    table.header.stamp = ros::Time::now();
    table.header.frame_id = robot_frame;
    
    Eigen::Matrix4f tfmat = getHomogeneousMatrix(camera_optical_frame,robot_frame);

    //people_detection::PersonObject pers;
    Eigen::Vector4f pubpts;
    
    //pubpts << tracklist[i].points(0),tracklist[i].points(1),tracklist[i].points(2),1.0;
    pubpts << table.pose.position.x, table.pose.position.y, table.pose.position.z, 1.0;
    pubpts = tfmat*pubpts;
    msg.centriod.x = pubpts(0);
    msg.centriod.y = pubpts(1);
    msg.centriod.z = pubpts(2);
  }

private:
  ros::NodeHandle nh; 
  ros::Subscriber table_sub;
  ros::Publisher marker_pub;
  //ros::ServiceServer solid_shape_serv;
  ros::Publisher solid_shape_pub;
  object_recognition_msgs::TableArray table_array;
  object_recognition_msgs::Table table;
  double size_x;
  double size_y;
  geometry_msgs::Point centroid;
  geometry_msgs::Pose table_pose;
  ros::Publisher pub_co;
  tf::TransformListener* listener;
  object_detection::ObjectDetection msg;
};


// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "table_shapes");
  ROS_INFO("table_shapes starting ... ^^");
  TableShape table_shape;
  ros::spin();
  return 0;
}
