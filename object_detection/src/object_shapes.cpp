#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <object_recognition_msgs/RecognizedObjectArray.h>
//#include <shape_msgs/SolidPrimitive.h>
#include <object_detection/ObjectDetection.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/CollisionObject.h>

class ObjectShape
{
public:
  ObjectShape()
  {
    object_sub = nh.subscribe("recognized_object_array", 1, &ObjectShape::objectCallback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("object_marker", 1);
    solid_shape_pub = nh.advertise<object_detection::ObjectDetection>("object_shape", 1);
    pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

    //solid_shape_pub = nh.advertise<shape_msgs::SolidPrimitive>("object_shape", 1);
    //solid_shape_serv = nh.advertiseService("object_shape", &ObjectShape::getSolidPrimitive, this);
  }


  void objectCallback(const object_recognition_msgs::RecognizedObjectArrayPtr& object_array)
  {
    //ROS_INFO("objectCallback");
    
    for (size_t index = 0; index < 1; ++index) 
    {
        object_recognition_msgs::RecognizedObject object = object_array->objects[index];
        std_msgs::Header object_header = object.header;
        //ROS_INFO("  header %s %f ", object_header.frame_id.c_str(), (double)object_header.seq);        

        object_pose = object.pose.pose.pose;
        getCloudMarker(object_pose);
        getSolidPrimitive();
        

      /*
      object_recognition_msgs/ObjectType type
      float32 confidence

      sensor_msgs/PointCloud2[] point_clouds
      shape_msgs/Mesh bounding_mesh
      geometry_msgs/Point[] bounding_contours

      geometry_msgs/PoseWithCovarianceStamped pose
      */

    }
  }

  void getCloudMarker(const geometry_msgs::Pose object_pose)
  {
      //ROS_INFO("getCloudMarker");
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/camera_depth_optical_frame";
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.stamp = ros::Time();

      marker.type = visualization_msgs::Marker::CUBE;

      marker.pose.position.x = object_pose.position.x + 0.05;
      marker.pose.position.y = object_pose.position.y - 0.05;
      marker.pose.position.z = object_pose.position.z - 0.1;
      marker.pose.orientation.x = object_pose.orientation.x;
      marker.pose.orientation.y = object_pose.orientation.y;
      marker.pose.orientation.z = object_pose.orientation.z;
      marker.pose.orientation.w = object_pose.orientation.w;
      
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.1;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
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

      geometry_msgs::Point point;
      point.x = object_pose.position.x;
      point.y = object_pose.position.y;
      point.z = object_pose.position.z;
      marker.points.push_back(point);

      /*
      for (size_t i = 0; i < object_contours.size(); i++) 
      {
          geometry_msgs::Point point;
          point.x = object_contours[i].x;
          point.y = object_contours[i].y;
          point.z = object_contours[i].z;
          marker.points.push_back(point);
      }
      ROS_INFO("object_contours %d",(int)object_contours.size());
      */
      marker_pub.publish(marker);

  }

  //bool getSolidPrimitive(using_markers::ObjectShape::Request &req, using_markers::ObjectShape::Response &res)
  void getSolidPrimitive()
  {
    ROS_INFO("Object Primitive Shape");
    shape_msgs::SolidPrimitive solid_shape;
    solid_shape.type = shape_msgs::SolidPrimitive::BOX;
    solid_shape.dimensions.resize(3);

    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
    solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
    
    //solid_shape.dimensions.push_back(0.3);
    //solid_shape.dimensions.push_back(0.3);
    //solid_shape.dimensions.push_back(0.3);
    //res.solid_shape = solid_shape;
    //res.centriod = object_pose;
    
    //solid_shape_serv.publish(solid_shape);
    //solid_shape_pub.publish(solid_shape);
    object_detection::ObjectDetection msg;
    msg.solid_shape = solid_shape;
    msg.centriod = object_pose.position;
    solid_shape_pub.publish(msg);

    // publish collision message to moveit
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";

    // remove object
    co.id = "object";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add object
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = solid_shape.type;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = solid_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = object_pose.position.x;
    co.primitive_poses[0].position.y = object_pose.position.y;
    co.primitive_poses[0].position.z = object_pose.position.z;
    co.primitive_poses[0].orientation.w = 1.0;
    pub_co.publish(co);


    //ROS_INFO("Table Plane Primitive Shape: [%d %.2f %.2f]", (int)res.solid_shape.type, size_x, size_y);
    //return true;
  }




private:
  ros::NodeHandle nh; 
  ros::Subscriber object_sub;
  object_recognition_msgs::RecognizedObjectArray object_array;
  ros::Publisher marker_pub;
  //ros::ServiceServer solid_shape_serv;
  ros::Publisher pub_co;
  ros::Publisher solid_shape_pub;
  geometry_msgs::Pose object_pose;
};

// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "object_shapes");
  ROS_INFO("object_shapes starting ... ^^");
  ObjectShape object_shape;
  ros::spin();
  return 0;
}
