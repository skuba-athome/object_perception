#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <object_recognition_msgs/TableArray.h>
//#include <shape_msgs/SolidPrimitive.h>
#include <object_detection/ObjectDetection.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/CollisionObject.h>

class TableShape
{
public:
  TableShape()
  {
    table_sub = nh.subscribe("table_array", 1, &TableShape::tableCallback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("table_marker", 1);
    solid_shape_pub = nh.advertise<object_detection::ObjectDetection>("table_shape", 1);
    pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    //solid_shape_pub = nh.advertise<shape_msgs::SolidPrimitive>("table_shape", 1);
    //solid_shape_serv = nh.advertiseService("table_shape", &TableShape::getSolidPrimitive, this);
  }

  void tableCallback(const object_recognition_msgs::TableArrayPtr& table_array)
  {
    //ROS_INFO("tableCallback");
    
    for (size_t index = 0; index < 1; ++index) 
    //for (std::vector<int>::iterator it = table_array.begin(); table_array.end(); it++)
    {
        object_recognition_msgs::Table table = table_array->tables[index];
        std_msgs::Header table_header = table.header;
        //geometry_msgs::Pose table_pose = table.pose;
        table_pose = table.pose;
        //ROS_INFO("POSE: [%f %f %f]", table_pose.position.x, table_pose.position.y, table_pose.position.z);
        
        std::vector<geometry_msgs::Point> table_convex_hull = table.convex_hull;
        //ROS_INFO("  table_convex_hull %d",(int)table_convex_hull.size());

        double xmax = 2.2250738585072014e-308;
        double xmin = 1.7976931348623158e+308;
        double ymax = 2.2250738585072014e-308;
        double ymin = 1.7976931348623158e+308;
        double z = 0;


        for (size_t i = 0; i < table_convex_hull.size(); i++) 
        {
          geometry_msgs::Point vertex;
          vertex.x = table_convex_hull[i].x;
          vertex.y = table_convex_hull[i].y;
          vertex.z = table_convex_hull[i].z;

          if(xmin>=vertex.x)
            xmin = vertex.x;
          if(ymin>=vertex.y)
            ymin = vertex.y;
          if(xmax<=vertex.x)
            xmax = vertex.x;
          if(ymax<=vertex.y)
            ymax = vertex.y;
          z = vertex.z;

          //ROS_INFO(" vertex: [%f %f %f]", vertex.x, vertex.y, vertex.z);  
        }
        //ROS_INFO(" min_max: [%f %f %f %f %f]", xmin, xmax, ymin, ymax, z);

        //double size_x = xmax-xmin;
        //double size_y = ymax-ymin;
        size_x = xmax-xmin;
        size_y = ymax-ymin;
        //ROS_INFO(" size: [%f %f]", xmax-xmin, ymax-ymin);
        //object_recognition_msgs::Table table = table_array->tables[*it];
        //visualization_msgs::Marker marker = getCloudMarker(table);
        //marker_table = getCloudMarker(table);

        //getCloudMarker(table_pose, size_x, size_y);
        getCloudMarker(table);
        getSolidPrimitive();
    }
  }

  void getCloudMarker(const object_recognition_msgs::Table table)
  //void getCloudMarker(const geometry_msgs::Pose table_pose, const double size_x, const double size_y)
  {
      //ROS_INFO("getCloudMarker");
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/camera_depth_optical_frame";
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.stamp = ros::Time();
      //marker.lifetimer = ros::Duration(5);
      
      marker.type = visualization_msgs::Marker::CUBE;

      marker.pose.position.x = table_pose.position.x;
      marker.pose.position.y = table_pose.position.y;
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
    ROS_INFO("Table Plane Primitive Shape");

    shape_msgs::SolidPrimitive solid_shape;
    solid_shape.type = shape_msgs::SolidPrimitive::BOX;
    solid_shape.dimensions.resize(3);
    //solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
    //solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.3;
    //solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size_x;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size_y;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    
    //solid_shape.dimensions.push_back(0.3);
    //solid_shape.dimensions.push_back(0.3);
    //solid_shape.dimensions.push_back(0.3);
    //res.solid_shape = solid_shape;
    //res.centriod = table_pose;
    
    object_detection::ObjectDetection msg;
    msg.solid_shape = solid_shape;
    msg.centriod = table_pose.position;
    solid_shape_pub.publish(msg);
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
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = table_pose.position.x;
    co.primitive_poses[0].position.y = table_pose.position.y;
    co.primitive_poses[0].position.z = table_pose.position.z;
    pub_co.publish(co);


    //ROS_INFO("Table Plane Primitive Shape: [%d %.2f %.2f]", (int)res.solid_shape.type, size_x, size_y);
    //return true;
  }

private:
  ros::NodeHandle nh; 
  ros::Subscriber table_sub;
  ros::Publisher marker_pub;
  //ros::ServiceServer solid_shape_serv;
  ros::Publisher solid_shape_pub;
  object_recognition_msgs::TableArray table_array;
  geometry_msgs::Pose table_pose;
  double size_x;
  double size_y;
  ros::Publisher pub_co;
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
