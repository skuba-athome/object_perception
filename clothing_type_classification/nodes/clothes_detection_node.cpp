#include <cstdlib>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "ClothesDetector.h"
#include <actionlib/server/simple_action_server.h>
#include <clothing_type_classification/FindClothesAction.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ClothesDetector.h"

#define DEFAULT_CLOUD_TOPIC "/camera/depth_registered/points"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ClothesDetectionRunner
{
	private:
		ros::NodeHandle nh;
		PointCloudT::Ptr cloud_obj;
        ros::Subscriber cloub_sub;
		actionlib::SimpleActionServer<clothing_type_classification::FindClothesAction> find_clothes_as_;
        clothing_type_classification::FindClothesActionFeedback find_clothes_feedback_;
        clothing_type_classification::FindClothesActionResult find_clothes_result_;

		//-------Clothes Detector Instance-------
		ClothesDetector clothes_detector;
	
	public:
		ClothesDetectionRunner(std::string node_name):
		nh("~"),
		find_clothes_as_(nh, node_name.c_str(), boost::bind(&ClothesDetectionRunner::executeFindClothesCallback, this, _1), false)
		{
			this->cloub_sub = nh.subscribe(DEFAULT_CLOUD_TOPIC, 1, &ClothesDetectionRunner::cloudCallback, this);
		};

		void executeFindClothesCallback(const clothing_type_classification::FindClothesGoalConstPtr &goal)
		{
			//TODO-- To Nuttaphon: Implement Your Extract Plane & EGBIS Code Here!!!
		}

		void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
		{
			pcl::fromROSMsg (*cloud_in, *cloud_obj);
		}

};

int main( int argc, char **argv )
{
    ros::init( argc, argv, "clothes_detection_node");
    ClothesDetectionRunner runner(ros::this_node::getName());
    ros::spin();
    return 0;
}	