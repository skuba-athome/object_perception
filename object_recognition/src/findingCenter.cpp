#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <object_recognition/classifyObject.h>
#include <object_recognition/verifyObject.h>
#include <object_recognition/Object.h>
#include <object_recognition/ObjectContainer.h>
#include <object_recognition/findObject.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

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

#include <sstream>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <ros/package.h>

using namespace std;

#define DEPTH_LIMIT 0.86
#define PLANE_LEFT 0.35
#define PLANE_RIGHT -0.35
#define ACCEPTED_AREA 300
//#define ACCEPTED_AREA 100
#define FOCAL_LENGTH 525

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

//void getObjectPoint();

cv::Mat img;
float pixel_x,pixel_y;
float objectCentroidWorld[20][3];

//std::string logitech_frame = "external_cam";
//std::string logitech_frame = "logitech_cam";
std::string robot_frame = "base_link";
std::string logitech_frame = "/camera_rgb_optical_frame";
//std::string output_dir = "/home/mukda/catkin_athome/src/object_recognition/object_perception/";
std::string output_dir = ros::package::getPath("object_recognition") + "/test/";

// service client
ros::ServiceClient classifyClient;
object_recognition::classifyObject classifySrv;

tf::TransformListener* listener;
ros::Time timeStamp, timeStamp_;

static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);

void depthCB(const sensor_msgs::PointCloud2& cloud) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 PCLPointCloud_tmp;

    //cout << "depthCB" << endl;

    if ( (cloud.width * cloud.height) == 0 )
    {
        cout << "return if the cloud is not dense!" << endl;
        return; 
    }
    try {
        pcl_conversions::toPCL(cloud, PCLPointCloud_tmp);
        pcl::fromPCLPointCloud2(PCLPointCloud_tmp, *cloud_tmp);
        //frameId = cloud.header.frame_id;
        timeStamp = cloud.header.stamp;
        //cout << "cloud.header " << cloud.header << endl;
        //cout << "timeStamp " << timeStamp << endl;
        
        //pcl::fromROSMsg(cloud, *cloud_pcl);
        //pcl::fromROSMsg(cloud, *cloud_tmp);
        int n = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_tmp->begin(); it != cloud_tmp->end(); ++it)
        {
            float x = it->x;
            float y = it->y;
            float z = it->z;
            //cout << "(" << x << "," << y << "," << z << ")" << endl;

            // if the object is on the table
            if( x < DEPTH_LIMIT && y < PLANE_LEFT && y > PLANE_RIGHT )//&& z > PLANE_HEIGHT)
            {
                cloud_tmp2->push_back(pcl::PointXYZ(it->x,it->y,it->z));
                n++;
            }
            //if( sqrt(x*x+y*y+z*z) < ARM_RAIDUS){
            //    cloud_tmp2->push_back(pcl::PointXYZ(it->x,it->y,it->z));
            //}
        }
        cloud_tmp2->header = cloud_tmp->header;
        //cout << "cloud_tmp2->header " << cloud_tmp2->header << endl;
        //cout << "first cloud size : " << cloud_tmp->width*cloud_tmp->height << " ,modified cloud size : " << cloud_tmp2->width*cloud_tmp2->height << endl;
        //        cloud_tmp2->width = 1;
        //        cloud_tmp2->height = n;
        //cout << "width : " << cloud_tmp2->width << " height : " << n << endl;
        
///        listener->waitForTransform(logitech_frame, cloud.header.frame_id, cloud.header.stamp, ros::Duration(1.0));

        //listener->waitForTransform(logitech_frame, cloud.header.frame_id, cloud.header.stamp, ros::Duration(2.0));
        //cout << "before transformPointCloud" << endl;
///        pcl_ros::transformPointCloud(logitech_frame, *cloud_tmp2, *cloud_pcl, *listener);

        cloud_pcl = cloud_tmp2;
        //cout << "cloud_pcl " << cloud_pcl->width*cloud_pcl->height << endl;

        //getObjectPoint();
        //cout << "cloud_pcl->header.frame_id" << cloud_pcl->header.frame_id << endl;
        //ROS_INFO("Get PointCloud size : %d",cloud.width*cloud.height);
        //  cout << "point cloud get" << endl;
        } 
    catch (std::runtime_error e) {
            ROS_ERROR_STREAM("Error message: " << e.what());
        }
    
}

//void getObjectPoint(object_recognition::findObject::Response &res)
bool getObjectPoint(object_recognition::findObject::Request &req, object_recognition::findObject::Response &res)
//void getObjectPoint()
{
    //cout << "getObjectPoint" << endl;
    timeStamp_ = timeStamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);

    int objectNumber=0;    
    std::stringstream center_ss;

    cloud = cloud_pcl;
    //Read in the cloud data
    //pcl::PCDReader reader;
    //reader.read ("1389374538.210053381.pcd", *cloud);
    //cout<< "cloud " << cloud->width*cloud->height << endl;

    vector<std::string> fileName;
    vector<int> compression_params; //vector that stores the compression parameters of the image

    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //specify the compression technique    
    compression_params.push_back(9); //specify the compression quality

    bool bSuccess_ = imwrite(output_dir+"first_img.jpg", img, compression_params); //write the image to file
    
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (output_dir+"first_cloud_pcl.pcd", *cloud, false); 

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);   
    vg.setLeafSize (0.002f, 0.002f, 0.002f);
    //vg.setLeafSize (0.001f, 0.001f, 0.001f);
    //vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*cloud_filtered);

    //std::cout << "PointCloud(cloud) before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
    //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    if(cloud_filtered->size() != 0 )
///     writer.write<pcl::PointXYZ> ("/run/shm/object_perception/cloud_filtered.pcd", *cloud_filtered, false); 
        writer.write<pcl::PointXYZ> (output_dir+"cloud_filtered.pcd", *cloud_filtered, false); 

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    int segmented_plane_no =0;
    //while (cloud_filtered->points.size () > 0.1 * nr_points)
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    //while (cloud_filtered->points.size () > 0.7 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        /*
        std::vector< float > coef = coefficients->values;
        for(int j=0;j<coef.size();j++){
            ROS_INFO("%f ",coef.at(j));
        }
        ROS_INFO("\n");
        */

        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
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
///     plane_name << "/run/shm/object_perception/segmenged_plane" << j << ".pcd";
        plane_name << "segmenged_plane" << segmented_plane_no << ".pcd";
        writer.write<pcl::PointXYZ> (output_dir+plane_name.str (), *cloud_plane, false); 
        segmented_plane_no++;
    }


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm   
    ec.setMinClusterSize (ACCEPTED_AREA);
    //ec.setMinClusterSize (200);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
    cout << "cluster_indeces.size() = " << cluster_indices.size() << endl;

    // finding center of object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center (new pcl::PointCloud<pcl::PointXYZ>);
    int objectNum=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {   
        float x=0,y=0,z=0;
        float pixel_x_max = 0, pixel_y_max = 0;
        float pixel_x_min = CENTER_IMAGE_X*2, pixel_y_min = CENTER_IMAGE_Y*2;

        stringstream pixelFileName;
        pixelFileName << output_dir << "file_pixel" << objectNumber;
        //cout << pixelFileName.str() << endl;

        FILE* fp = fopen(pixelFileName.str().c_str(),"w");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            float tmpX,tmpY;
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
            x += cloud_filtered->points[*pit].x;
            y += cloud_filtered->points[*pit].y;
            z += cloud_filtered->points[*pit].z;

            //for webcam 1280x720
            tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_X;
            tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH_Y/cloud_filtered->points[*pit].z + CALIBRATED_CENTER_IMAGE_Y;// + 20;

            if(tmpX > pixel_x_max) pixel_x_max = tmpX;
            if(tmpY > pixel_y_max) pixel_y_max = tmpY;
            if(tmpX < pixel_x_min) pixel_x_min = tmpX;
            if(tmpY < pixel_y_min) pixel_y_min = tmpY;

            char pixelValue[100];
            sprintf(pixelValue,"(%f,%f,%f)  (%f,%f)\n",cloud_filtered->points[*pit].x,cloud_filtered->points[*pit].y,cloud_filtered->points[*pit].z,tmpX,tmpY);
            fputs (pixelValue,fp);

            //--------------------------------------------------------test--------------------------------------
            //tmpX = cloud_filtered->points[*pit].x*FOCAL_LENGTH_X/cloud_filtered->points[*pit].z + CENTER_IMAGE_X_;
            //tmpY = cloud_filtered->points[*pit].y*FOCAL_LENGTH_Y/cloud_filtered->points[*pit].z + CENTER_IMAGE_Y_;
            //if(tmpX > pixel_x_max_) pixel_x_max_ = tmpX;
            //if(tmpY > pixel_y_max_) pixel_y_max_ = tmpY;
            //if(tmpX < pixel_x_min_) pixel_x_min_ = tmpX;
            //if(tmpY < pixel_y_min_) pixel_y_min_ = tmpY;
        }
        fclose (fp);

        // save object model
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        stringstream ss;
        ss << output_dir << "cluster_" << objectNumber << ".pcd";        
        if(cloud_cluster->size() != 0 )
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);


        pixel_x_min = pixel_x_min > 0 ? pixel_x_min : 0;
        pixel_y_min = pixel_y_min > 0 ? pixel_y_min : 0;
        pixel_x_max = pixel_x_max < CENTER_IMAGE_X*2 ? pixel_x_max : CENTER_IMAGE_X*2;
        pixel_y_max = pixel_y_max < CENTER_IMAGE_Y*2 ? pixel_y_max : CENTER_IMAGE_Y*2;        

        int size = std::distance(it->indices.begin(), it->indices.end());
        x/=size;
        y/=size;
        z/=size;

        float objectWorldX,objectWorldY,objectWorldZ;        

        pixel_x = x*FOCAL_LENGTH_X/z + CALIBRATED_CENTER_IMAGE_X;// + (-0.5);
        pixel_y = y*FOCAL_LENGTH_Y/z + CALIBRATED_CENTER_IMAGE_Y;// + 20;
        center_ss << pixel_x << " " << pixel_y << " ";

        double topLeftX,topLeftY;
        topLeftX = max(pixel_x_min+TUNED_H_DISTANCE_TOP_LEFT,double(0.0));
        topLeftX = min(topLeftX,double(CENTER_IMAGE_X*2));
        topLeftY = max(pixel_y_min+TUNED_V_DISTANCE_TOP_LEFT,double(0.0));
        topLeftY = min(topLeftY,double(CENTER_IMAGE_Y*2));

        int width_,height_;
        if(pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT <= CENTER_IMAGE_X*2 )
            width_ = (pixel_x_max+TUNED_H_DISTANCE_BOTTOM_RIGHT)-(topLeftX);
        else
            width_ = max((double)(CENTER_IMAGE_X*2) -topLeftX,(double)0);            
        
        if(pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT <= CENTER_IMAGE_Y*2 )
            height_ = (pixel_y_max+TUNED_V_DISTANCE_BOTTOM_RIGHT)-(topLeftY);
        else
            height_ = max((double)(CENTER_IMAGE_Y*2) - topLeftY,(double)0);

        cout << "value of the cropped conner with fixing and tuning" << '(' << topLeftX << ',' << topLeftY << ") (" <<topLeftX + width_<< ',' << topLeftY + height_<< ')' << endl;
        cout << "width : "<< width_<<  " height : " << height_ << endl;

        if(pixel_x_max <= 0 || pixel_x_min >= CENTER_IMAGE_X*2 || pixel_y_max <= 0 || pixel_y_min >= CENTER_IMAGE_Y*2)
            continue;

        if(width_<=0 || height_ <=0)
            continue;

        
        objectCentroidWorld[objectNumber][0] = objectWorldX;
        objectCentroidWorld[objectNumber][1] = objectWorldY;
        objectCentroidWorld[objectNumber][2] = objectWorldZ;

        cv::Mat m = img.clone();
        IplImage* iplImage = new IplImage(m);
        cvSetImageROI(iplImage,cv::Rect(topLeftX,topLeftY,width_,height_));

        std::stringstream ss_;        
        ss_<< output_dir << "picture" << objectNumber << ".png";
        bool bSuccess = imwrite(ss_.str(), cv::Mat(iplImage), compression_params); //write the image to file

        fileName.push_back(ss_.str());

        cloud_center->push_back(pcl::PointXYZ(x,y,z));

        ROS_INFO("(object's point in RGB (%f,%f)\n",pixel_x,pixel_y);
        ROS_INFO("TOPLEFT : (%f,%f) TOP_RIGHT : (%f,%f)\n",pixel_x_min,pixel_y_min,pixel_x_max,pixel_y_max);
        ROS_INFO("center of clustering no.%d : (%f,%f,%f) \n",objectNumber,x,y,z);

        objectNumber++;
    }

    // classify object call service 
    for(int k=0;k<objectNumber;k++)
    {
        classifySrv.request.filepath = fileName[k];
        //verifySrv.request.objectPictureFilePath = fileName[k];
        cout << "path : " << fileName[k] << endl;

        if(classifyClient.call(classifySrv))
        {
            //cout << "start classify"  << endl;
            /*
            object_recognition::Object object;
            object.point.x = objectCentroidWorld[k][0];
            object.point.y = objectCentroidWorld[k][1];
            object.point.z = objectCentroidWorld[k][2];
            object.category = classifySrv.response.objectCategory;
            object.confident = classifySrv.response.confident;
            */
            
            cout << "response from server : category = " << classifySrv.response.objectCategory << " confidient = " << classifySrv.response.confident  << endl;
            cout << "position of centroid : " << objectCentroidWorld[k][0] << " " << objectCentroidWorld[k][1] << " " <<objectCentroidWorld[k][2] << " " <<endl;
            
            res.objectName.push_back(classifySrv.response.objectCategory);
            res.confident.push_back(classifySrv.response.confident);

      		geometry_msgs::Point point;
      		point.x = objectCentroidWorld[k][0];
            point.y = objectCentroidWorld[k][1];
            point.z = objectCentroidWorld[k][2];
      		res.objectPos.push_back(point);

            //res.objectPositionXYZ.x.push_back(objectCentroidWorld[k][0]);
            //res.objectPositionXYZ.y.push_back(objectCentroidWorld[k][1]);
            //res.objectPositionXYZ.z.push_back(objectCentroidWorld[k][2]);
            //objectContainer.objects.push_back(object);
        }
        else{
            cout << "no response from server" << endl;
        }
    }

    //create center of object cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr enlarged_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t t = 0; t < cloud_center->size(); t++){
        for(float i=-0.007;i<=0.007;i+=0.001){
            for(float j=-0.007;j<=0.007;j+=0.001){
                for(float k=-0.007;k<=0.007;k+=0.001){
                    enlarged_cloud->push_back(pcl::PointXYZ(cloud_center->points[t].x+i,cloud_center->points[t].y+j,cloud_center->points[t].z+k));
                }
            }
        }
    }

    if(enlarged_cloud->size() != 0 )
        writer.write<pcl::PointXYZ> (output_dir+"centerOfObject.pcd", *enlarged_cloud, false); 

    return true;
}

void imageColorCb(const sensor_msgs::ImageConstPtr& msg)
{
    //cout << "imageColorCb" << endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;
        //cv::namedWindow("window", 1);
        //cv::imshow("window",img);
        //cv::waitKey(3);
        //ROS_INFO("Get image size : %d",img.rows*img.cols);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}



int main (int argc, char** argv)
{
	ros::init(argc,argv,"finding_CenterTest");
	ros::NodeHandle n;	
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ROS_INFO("Start finding center of the object ...");
    nh.param("FOCAL_LENGTH_X", FOCAL_LENGTH_X, 814.03512);
    nh.param("FOCAL_LENGTH_Y", FOCAL_LENGTH_Y, 815.46674);
    nh.param("CALIBRATED_CENTER_IMAGE_X", CALIBRATED_CENTER_IMAGE_X,313.73619);
    nh.param("CALIBRATED_CENTER_IMAGE_Y", CALIBRATED_CENTER_IMAGE_Y,254.26251);
    nh.param("CENTER_IMAGE_X", CENTER_IMAGE_X,320.0);
    nh.param("CENTER_IMAGE_Y", CENTER_IMAGE_Y,240.0);
    //nh.param("TUNED_H_DISTANCE_TOP_LEFT", TUNED_H_DISTANCE_TOP_LEFT,0.0);
    nh.param("TUNED_H_DISTANCE_TOP_LEFT", TUNED_H_DISTANCE_TOP_LEFT,0.0);
    nh.param("TUNED_V_DISTANCE_TOP_LEFT", TUNED_V_DISTANCE_TOP_LEFT,0.0);
    nh.param("TUNED_H_DISTANCE_BOTTOM_RIGHT", TUNED_H_DISTANCE_BOTTOM_RIGHT,0.0);
    nh.param("TUNED_V_DISTANCEw_BOTTOM_RIGHT", TUNED_V_DISTANCE_BOTTOM_RIGHT,0.0);
    
    classifyClient = n.serviceClient<object_recognition::classifyObject>("classifyObject");
    
    image_transport::Subscriber sub_imageColor = it.subscribe("/external_cam/image_raw", 1, imageColorCb);
    //image_transport::Subscriber sub_imageColor = it.subscribe("/logitech_cam/image_raw", 1, imageColorCb);
    //image_transport::Subscriber sub_imageColor = it.subscribe("/camera/rgb/image_color", 1, imageColorCb);

    ros::Subscriber sub = n.subscribe("/depth_registered/depth_registered/points",1, depthCB);
    //ros::Subscriber sub = n.subscribe("/cloud_tf",1, depthCB);
    //ros::Subscriber sub = n.subscribe("/camera/depth_registered/points",1,depthCB);

    ros::ServiceServer service = n.advertiseService("findObject", getObjectPoint);
//    listener = new tf::TransformListener();

	ros::spin();

	return (0);
}