#include <iostream>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" 
#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h> 
#include <math.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/vfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/String.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Vector3.h>
#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#define distance_ratio 100
#define ACCEPTED_DISTANCE 50
#define AGGREGRATED_FRAME 3
#define EXIST_POSITION 1
#define ACCEPTED_CONTOUR_AREA 300
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using namespace boost;
enum State { START,TRACKING,NORMAL,FIND_REGION};
Mat image;
bool isStart= false,regionExist=false;
IplImage* img;
IplImage* imgTresh;
IplImage* imgHsv;
typedef struct my_contour{
	vector<vector<Point> > my_contours;
	vector<Vec4i> my_hierarchy; 
	int max_index;
} my_contour;
my_contour* current_contour;
my_contour* new_contour;
bool isFirst = true;
bool trigger = false;
int state = START,substate=NORMAL;
int imgHeight,imgWidth,center_x,center_y;
int current_x,current_y,sum_x,sum_y,frame_count=0,old_position,new_position,old_max_index;
int countTmp=0,countJa=0;
int global_argc;
int lowerH=0;
int lowerS=0;
int lowerV=0;
int upperH=180;
int upperS=256;
int upperV=256;
int divider =0;
char vertical_last_cmd[5]="s",vertical_cmd[5]="s";
static const char WINDOW[] = "Image";
ros::Publisher vector_pub;
ros::Publisher vector_pub_pointcloud;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
bool findColor();
void sendPosition(int x , int y);
void get_contour_center(int* x,int* y);
void draw(int maxIndex,vector<vector<Point> > contours,vector<Vec4i> hierarchy);
void filterContour();
double distance(int a,int b,int c,int d);
void update_current_contour();
void setwindowSetting();
float global_x = 0,global_y =0,global_z = 0,send_x=0,send_y=0,send_z=0;
void setwindowSettings(){
cvNamedWindow("Color");
cvCreateTrackbar("LowerH", "Color", &lowerH, 180, NULL);
        cvCreateTrackbar("UpperH", "Color", &upperH, 180, NULL);
cvCreateTrackbar("LowerS", "Color", &lowerS, 256, NULL);
        cvCreateTrackbar("UpperS", "Color", &upperS, 256, NULL);
cvCreateTrackbar("LowerV", "Color", &lowerV, 256, NULL);
        cvCreateTrackbar("UpperV", "Color", &upperV, 256, NULL);
}
void transformer(int x, int y)
{
 //   printf("%d %d\n",x,y);
	if ((cloud_pcl->width * cloud_pcl->height) == 0)
    			return;
		geometry_msgs::Vector3 vector;
		vector.x = cloud_pcl->points[y*640+x].x;
		vector.y = cloud_pcl->points[y*640+x].y;
		vector.z = cloud_pcl->points[y*640+x].z;
                if( vector.x == vector.x 
			&& vector.y == vector.y
			&& vector.z == vector.z
                  )
                {
                global_x += vector.x;
                global_y += vector.y;
                global_z += vector.z;
	                    //printf("x:%f y:%f z:%f\n",vector.x,vector.y,vector.z);
                }
                else 
                    divider++;

}
void depthCb(const sensor_msgs::PointCloud2& cloud) {
  if ((cloud.width * cloud.height) == 0)
      return;
  try {
    pcl::fromROSMsg(cloud, *cloud_pcl);
  } catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Error message: " << e.what());
  }
}
void sendPosition(float x , float y ,float z){
			geometry_msgs::Vector3 vector;
			vector.x = x;
			vector.y = y;
			vector.z = z;
	        if(vector.x == vector.x && vector.y == vector.y && vector.z == vector.z)
            vector_pub.publish(vector);
			//printf("send : x:%.2f y:%.2f z:%.2f\n",vector.x,vector.y,vector.z);						
		
}

void filterContour(){
	if(!regionExist){
		current_x = center_x;
		current_y = center_y;
		return;
	}
	if(state==START){
		update_current_contour();
		state = TRACKING;
	}
	else if(state==TRACKING){
		if(substate==NORMAL){
			int new_contour_x=0,new_contour_y=0;
			get_contour_center(&new_contour_x,&new_contour_y);
			if(distance(current_x,current_y,new_contour_x,new_contour_y) < distance_ratio){
				update_current_contour();
				get_contour_center(&current_x,&current_y);
			}
			else{
				substate = FIND_REGION;
				sum_x = 0;
				sum_y = 0;
				old_position=0;
				new_position=0;
			}
		}
		else if(substate==FIND_REGION){
			int tmp_x,tmp_y;
			get_contour_center(&tmp_x,&tmp_y);
			if(distance(current_x,current_y,tmp_x,tmp_y) < distance_ratio)
				old_position++;
			else
				new_position++;
			frame_count++; 
			if(old_position > EXIST_POSITION){
				frame_count=0;
				substate = NORMAL;
				return;
			}
			if(frame_count==AGGREGRATED_FRAME){
				update_current_contour();
				get_contour_center(&current_x,&current_y);
				frame_count=0;
				substate = NORMAL;
			}
		}
	}
}
void get_contour_center(int* x,int* y){
	if(new_contour->my_contours.size() == 0) return;
	int index,sizes = 0;
	float sum_contour_z=0,sum_contour_y=0,sum_contour_x=0;
	index = new_contour->max_index;
	sizes = new_contour->my_contours.at(index).size();
    divider = 0;
	for(int i=0;i<sizes;i++){
		transformer(new_contour->my_contours.at(index).at(i).x , new_contour->my_contours.at(index).at(i).y);
        }
        //printf("%d\n",divider);
        sum_contour_x = global_x/(sizes-divider);//Protect nan
        sum_contour_y = global_y/(sizes-divider);
        sum_contour_z = global_z/(sizes-divider);
        if(sum_contour_x != 0 && sum_contour_y != 0 && sum_contour_z != 0)
        {
            send_x = sum_contour_x;
            send_y = sum_contour_y;
            send_z = sum_contour_z;
        }
        divider = 0;
        global_x = 0;
        global_y = 0;
        global_z = 0;
		//cout<<sum_contour_x<<" "<< sum_contour_y<<" "<<sum_contour_z<<endl;
}
double distance(int a,int b,int c,int d){
	return sqrt((a-c)*(a-c) + (b-d)*(b-d));
}
void update_current_contour(){
	current_contour->my_contours = new_contour->my_contours;
	current_contour->my_hierarchy = new_contour->my_hierarchy;
	current_contour->max_index = new_contour->max_index;
}
void draw(int maxIndex,vector<vector<Point> > contours,vector<Vec4i> hierarchy){
	Scalar color = Scalar(153,255,0);
	drawContours( Mat(imgTresh), contours, maxIndex, color, 2, 8, hierarchy, 0, Point() );
	drawContours( Mat(img), contours, maxIndex, color, 2, 8, hierarchy, 0, Point() );
	Mat m = Mat(img);
	int center_contour_x,center_contour_y;
	get_contour_center(&center_contour_x,&center_contour_y);
	if(center_contour_y==0&&center_contour_x==0){
		center_contour_x = current_x;
		center_contour_y = current_y;
	}
	if(regionExist){
		cv::circle(m, cv::Point(center_contour_x, center_contour_y), ACCEPTED_DISTANCE, CV_RGB(100,100,0));
               sendPosition(send_x,send_y,send_z);
	}
	cv::circle(m, cv::Point(center_x, center_y), 10, CV_RGB(255,0,0));
	cvNamedWindow("after draw contours");
	cvShowImage("after draw contours",imgTresh);
	cvNamedWindow("source");
	cvShowImage("source",img);
}
bool findColor(){
	Mat frame = image;
	img = new IplImage(frame);
	if(!isStart){
		imgHsv =cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
		imgHeight = frame.cols;
		center_x =imgHeight/2;
		imgWidth = frame.rows;
		center_y =imgWidth/2;
	}
	cvCvtColor(img,imgHsv,CV_BGR2HSV);
	if(!isStart){
		imgTresh=cvCreateImage(cvGetSize(imgHsv),IPL_DEPTH_8U,1);
		isStart = true;
	}
	cvSmooth(imgTresh,imgTresh,CV_GAUSSIAN,3,3);
	cvInRangeS(imgHsv,cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV) , imgTresh);
	cvDilate(imgTresh,imgTresh,NULL,3);
	cvErode(imgTresh,imgTresh);
	cvCanny( imgTresh, imgTresh, 100, 200, 3 );
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	findContours( Mat(imgTresh), contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	int maxIndex = 0; 
	for(int i=0;i<contours.size();i++){
		if(contourArea(contours.at(i)) > contourArea(contours.at(maxIndex)))
			maxIndex = i;
	}
	if(contours.size()!=0 && contourArea(contours.at(maxIndex)) > ACCEPTED_CONTOUR_AREA){
		new_contour->my_contours = contours;
		new_contour->my_hierarchy = hierarchy;
		new_contour->max_index = maxIndex;
		regionExist = true;
	}
	else
		regionExist = false;
	filterContour();
	draw(current_contour->max_index,current_contour->my_contours,current_contour->my_hierarchy);
}
void imageCallback(const sensor_msgs::Image::ConstPtr& img_in)
{	
	cv_bridge::CvImagePtr cv_ptr;
		try{
        		cv_ptr = cv_bridge::toCvCopy(img_in, enc::BGR8);
			image = cv_ptr -> image;
    		}
    		catch (cv_bridge::Exception& e){
        		ROS_ERROR("cv_bridge exception: %s", e.what());
        		return;
    		}
	trigger = findColor();
 	//cv::imshow(WINDOW, image);
	cv::waitKey(3);
}
int main (int argc, char** argv)
{
	int i = 0;
        int config[6];
        for(int x = 0 ;x<6;x++)
        config[x] = 0;

	string line;
	vector <string> fields;  	
	ifstream myfile ("/home/skuba/skuba_athome/object_perception/color_detection/bin/color_config.txt");
  	
	if (myfile.is_open())
  	{
    		while ( getline (myfile,line) )
    		{
			split( fields, line, is_any_of( ":" ) );
			config[i] = atoi(fields[1].c_str());
    			i++;
		}
    		myfile.close();
  	}
  	else cout << "Unable to open file"; 
	lowerH=config[0];
	lowerS=config[1];
	lowerV=config[2];
	upperH=config[3];
	upperS=config[4];
	upperV=config[5];
	setwindowSettings();
	current_contour = new my_contour();
	new_contour = new my_contour();
	ros::init(argc, argv, "color_detector");
	ros::NodeHandle n;
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	ros::Subscriber subDepth = n.subscribe("/cloud_tf",1,depthCb);
	//ros::Subscriber subDepth = n.subscribe("/camera/depth_registered/points",1,depthCb);
	ros::Subscriber	image_sub = n.subscribe("/camera/rgb/image_color", 1, imageCallback);
	vector_pub = n.advertise<geometry_msgs::Vector3>("color_detect", 1000);
	cv::destroyWindow(WINDOW);
	ros::spin(); 
  return 0;
}
