#include <cstdlib>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "ClothesDetector.h"


class ClothesDetectionRunner
{
	private:
		ros::NodeHandle nh;
	
	public:
		ClothesDetectionRunner():
		nh("~")
		{

		};

};

int main( int argc, char **argv )
{
    ros::init( argc, argv, "clothes_detection_node");
    ClothesDetectionRunner runner;
    ros::spin();
    return 0;
}	