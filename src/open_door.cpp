#include"open_door.h"

void depthCb( const sensor_msgs::ImageConstPtr& image )
{
	canPrintDepth = 0;
    try
    {
        bridge = cv_bridge::toCvCopy(image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform depth image.");
        return;
    }
    depthImg = Mat(bridge->image.rows, bridge->image.cols, CV_8UC1);
    for(int i = 0; i < bridge->image.rows; i++)
    {
        float* Di = bridge->image.ptr<float>(i);
        char* Ii = depthImg.ptr<char>(i);
        for(int j = 0; j < bridge->image.cols; j++)
        {
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
            dist[i][j] = Di[j];
        }
    }

    if(dist[240][320] > 2.0f || dist[240][320] != dist[240][320])
    {
        std_msgs::String msg;

        std::stringstream ss;

        ss << "open";

        msg.data = ss.str();

    	ROS_INFO("door is open !");
    	pub.publish(msg);

    	exit(0);
    }
    canPrintDepth = 1;
}


int main(int argc,char * argv[])
{
  FILE * imgListFile = 0;
  ros::init(argc,argv,"faces");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  nh.param("min_range", min_range_, 0.5);
  nh.param("max_range", max_range_, 5.5);
  ros::Subscriber subDepth = n.subscribe("/camera/depth/image",1,depthCb);
  pub = n.advertise<std_msgs::String>("door_cmd", 1000);
  ros::spin();
}




