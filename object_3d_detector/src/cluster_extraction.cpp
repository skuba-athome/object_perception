#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <object_3d_detector/Object3DsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#define DEFAULT_CLOUD_TOPIC "/depth_registered/depth_registered/points"

class ClusterExtraction {

protected:
    ros::NodeHandle nodeHandle;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<object_3d_detector::Object3DsAction> server;
    std::string name;
    object_3d_detector::Object3DsFeedback feedback;
    object_3d_detector::Object3DsResult result;
    tf::TransformListener listener;
    ros::Subscriber pointCloud2Subscriber;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    pcl::PCDWriter writer;
    std::stringstream path;
    std_msgs::Header header;
    float down_y_plane;
    float up_y_plane;
    float right_x_plane;
    float left_x_plane;
    float pos_z_plane;
    float neg_z_plane;
    bool  flag;

    void reset() {
        this->path.str("");
        this->path << ros::package::getPath("object_3d_detector");
        this->path << "/history/" << ros::Time::now() << "/";
        std::cout << this->path.str() << std::endl;
        std::stringstream path_;
        path_ << "mkdir -p " << this->path.str();
        const int dir_err = system(path_.str().c_str());
        down_y_plane = 9999;
        up_y_plane = -9999;
        right_x_plane = -9999;
        left_x_plane = 9999;
        pos_z_plane = -9999;
        neg_z_plane = 9999;
        ROS_INFO("ClusterExtraction RESET");
    }

    std_msgs::Header generateHeader(){
        this->header.seq++;
        this->header.stamp = ros::Time::now();
        this->header.frame_id = "base_link";
        return this->header;
    }


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        ROS_INFO("ClusterExtraction DOWNSAMPLE");
        pcl::VoxelGrid <pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filtered);
        //ROS_INFO("Saved: %s%s", this->path.str().c_str(), "cloud.pcd");
        //writer.write<pcl::PointXYZ>(this->path.str() + "cloud.pcd", *cloud, false);
        ROS_INFO("Saved: %s%s", this->path.str().c_str(), "cloud_filtered.pcd");
        writer.write<pcl::PointXYZ>(this->path.str() + "cloud_filtered.pcd", *cloud_filtered, false);
//        delete *vg;
        return cloud_filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        ROS_INFO("ClusterExtraction PASS_THROUGH_Z");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PassThrough <pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.1);
        pass.filter(*cut_cloud);
        if (not cut_cloud->empty()) {
            writer.write<pcl::PointXYZ>(this->path.str() + "cloud_pass_through_z.pcd", *cut_cloud, false);
            ROS_INFO("Saved: %s%s", this->path.str().c_str(), "cloud_pass_through_z.pcd");
        }
//        delete pass;
        return cut_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeNormalPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        ROS_INFO("ClusterExtraction FIND NORMAL_PLANE");
        pcl::SACSegmentationFromNormals <pcl::PointXYZ, pcl::Normal> seg;
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        pcl::ExtractIndices <pcl::PointXYZ> extract;
        pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);

        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);


        seg.setNormalDistanceWeight(0.1);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setProbability(0.99);
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud_normals);

        seg.segment(*inliers_plane, *coefficients_plane);

//        delete seg;

        extract.setInputCloud(cloud);
        extract.setIndices(inliers_plane);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*cloud_plane);

//        float down_y_plane=9999;
//        float up_y_plane=-9999;
//        float right_x_plane=-9999;
//        float left_x_plane=9999;
//        float pos_z_plane=-9999;
//        float neg_z_plane=9999;

        for (int i = 0; i < cloud_plane->points.size(); i++) {
            float x = std::abs(cloud_plane->points[i].x);
            float y = std::abs(cloud_plane->points[i].y);
            float z = std::abs(cloud_plane->points[i].z);

            left_x_plane = std::min(left_x_plane, cloud_plane->points[i].x);
            down_y_plane = std::min(down_y_plane, cloud_plane->points[i].y);
            neg_z_plane = std::min(neg_z_plane, cloud_plane->points[i].z);

            right_x_plane = std::max(right_x_plane, cloud_plane->points[i].x);
            up_y_plane = std::max(up_y_plane, cloud_plane->points[i].y);
            pos_z_plane = std::max(pos_z_plane, cloud_plane->points[i].z);
        }
        std::cout << "width_plane  " << std::abs(left_x_plane - right_x_plane) << std::endl;
        std::cout << "height_plane   " << std::abs(down_y_plane - up_y_plane) << std::endl;
        std::cout << "depth_plane   " << std::abs(neg_z_plane - pos_z_plane) << std::endl;
        std::cout << "down_y_plane  " << down_y_plane << std::endl;
        std::cout << "up_y_plane  " << up_y_plane << std::endl;
        std::cout << "right_x_plane  " << right_x_plane << std::endl;
        std::cout << "left_x_plane  " << left_x_plane << std::endl;
        std::cout << "pos_z_plane  " << pos_z_plane << std::endl;
        std::cout << "neg_z_plane  " << neg_z_plane << std::endl;

        if (not cloud_plane->empty()) {
            writer.write<pcl::PointXYZ>(this->path.str() + "cloud_plane.pcd", *cloud_plane, false);
            ROS_INFO("Saved: %s%s", this->path.str().c_str(), "cloud_plane.pcd");
        }
        ROS_INFO("ClusterExtraction CUT_NORMAL_PLANE");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*cloud_filtered);

        if (not cloud_filtered->empty()) {
            writer.write<pcl::PointXYZ>(this->path.str() + "cloud_remove_plane.pcd", *cloud_filtered, false);
            ROS_INFO("Saved: %s%s", this->path.str().c_str(), "cloud_remove_plane.pcd");
        }
//        pcl::ExtractIndices<pcl::Normal> extract_normals;
//        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
//
//        extract.setNegative (true);
//        extract.filter (*cloud_filtered2);
//        extract_normals.setNegative (true);
//        extract_normals.setInputCloud (cloud_normals);
//        extract_normals.setIndices (inliers_plane);
//        extract_normals.filter (*cloud_normals2);
        return cloud_filtered;
    }

    object_3d_detector::Object3DsResult::Ptr findObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        ROS_INFO("ClusterExtraction FIND_OBJECTS");
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector <pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction <pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        object_3d_detector::Object3DsResult::Ptr object3DsResult(new object_3d_detector::Object3DsResult);
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud->points[*pit]); //*
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." <<
            std::endl;
            // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.getHeight() << " data points." << std::endl;

            float down_y = 9999;
            float up_y = -9999;
            float right_x = -9999;
            float left_x = 9999;
            float pos_z = -9999;
            float neg_z = 9999;

            for (int i = 0; i < cloud_cluster->width; i++) {
                float x = std::abs(cloud_cluster->points[i].x);
                float y = std::abs(cloud_cluster->points[i].y);
                float z = std::abs(cloud_cluster->points[i].z);
                // std::cout << "TEST" << cloud_cluster->points[i] << std::endl;

                left_x = std::min(left_x, cloud_cluster->points[i].x);
                down_y = std::min(down_y, cloud_cluster->points[i].y);
                neg_z = std::min(neg_z, cloud_cluster->points[i].z);

                right_x = std::max(right_x, cloud_cluster->points[i].x);
                up_y = std::max(up_y, cloud_cluster->points[i].y);
                pos_z = std::max(pos_z, cloud_cluster->points[i].z);
            }
            std::cout << "width  " << std::abs(left_x - right_x) << std::endl;
            std::cout << "height  " << std::abs(down_y - up_y) << std::endl;
            std::cout << "depth  " << std::abs(neg_z - pos_z) << std::endl;
            std::cout << "down_y  " << down_y << std::endl;
            std::cout << "up_y  " << up_y << std::endl;
            std::cout << "right_x  " << right_x << std::endl;
            std::cout << "left_x  " << left_x << std::endl;
            std::cout << "pos_z " << pos_z << std::endl;
            std::cout << "neg_z  " << neg_z << std::endl;
            std::stringstream ss;
//            std::vector<> points;
//            if ((x_max - x_min) < 0.4 && (y_max - y_min) < 0.3 && (z_max - z_min) < 1) {
            std::cout << j << (pos_z_plane >= neg_z) << (neg_z_plane <= neg_z) << (left_x_plane <= left_x) <<
            (right_x_plane >= right_x) << std::endl;
            if (pos_z_plane >= neg_z
                // && neg_z_plane <= neg_z
                && left_x_plane <= left_x
                && right_x_plane >= right_x
                && std::abs(left_x - right_x) <= 0.15) {
                object_3d_detector::Object3D::Ptr object(new object_3d_detector::Object3D);
                object->header = generateHeader();
                geometry_msgs::PointStamped pointStamped;
                geometry_msgs::PointStamped pointStamped_;
                pointStamped.header.frame_id = "external_cam";
                pointStamped.header.stamp = ros::Time(0);
                pointStamped.point.x = (right_x + left_x)/2;
                pointStamped.point.y = (up_y + down_y)/2;
                pointStamped.point.z = (pos_z + neg_z)/2;
                listener.transformPoint("base_link", pointStamped, pointStamped_);

//                object->point.x = (right_x + left_x)/2;
//                object->point.y = (up_y + down_y)/2;
//                object->point.z = (pos_z + neg_z)/2;
                object->point.x = pointStamped_.point.x;
                object->point.y = pointStamped_.point.y;
                object->point.z = pointStamped_.point.z;
                object->width.data = std::abs(left_x - right_x);
                object->height.data = std::abs(down_y - up_y);
                object->depth.data = std::abs(neg_z - pos_z);
                object3DsResult->objects.push_back(*object);
                ss << "cloud_cluster_" << j << ".pcd";
                writer.write<pcl::PointXYZ>(this->path.str()+ss.str(), *cloud_cluster, false); //*
                ROS_INFO("Saved: %s%s", this->path.str().c_str(), ss.str().c_str());
            } else {
                ss << "No-cloud_cluster_" << j << ".pcd";
                writer.write<pcl::PointXYZ>(this->path.str()+ss.str(), *cloud_cluster, false); //*
            }
//            }
            j++;
        }
        return object3DsResult;

    }

    void receivePoinCloud2(const sensor_msgs::PointCloud2ConstPtr& cloud_in){
        ROS_INFO("ClusterExtraction RECEIVE_POINT_CLOUD_2");
        pcl::fromROSMsg (*cloud_in, *cloud);
        this->flag = true;
        this->pointCloud2Subscriber.shutdown();
    }


public:

    ClusterExtraction(std::string name) :
            server(nodeHandle, name, boost::bind(&ClusterExtraction::execute, this, _1), false),
            name(name),
            cloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        ROS_INFO("ClusterExtraction INIT");
        this->header.frame_id = "external_cam";
        server.start();
    }

    ~ClusterExtraction(void){

    }

    object_3d_detector::Object3DsResult::Ptr compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        this->reset();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = this->downsample(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through_z = this->passThroughZ(cloud_filtered);
        if (cloud_pass_through_z->empty()){
            object_3d_detector::Object3DsResult::Ptr object3DsResult(new object_3d_detector::Object3DsResult);
            return object3DsResult;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_plane = this->removeNormalPlane(cloud_pass_through_z);
        if (cloud_remove_plane->empty()){
            object_3d_detector::Object3DsResult::Ptr object3DsResult(new object_3d_detector::Object3DsResult);
            return object3DsResult;
        }
        object_3d_detector::Object3DsResult::Ptr result = this->findObjects(cloud_remove_plane);
        std::cout << *result << std::endl;
        return result;

    }

    void execute(const object_3d_detector::Object3DsGoalConstPtr & goal){

        pointCloud2Subscriber= nodeHandle.subscribe(DEFAULT_CLOUD_TOPIC, 1, &ClusterExtraction::receivePoinCloud2, this);
        while (!this->flag){
            ros::Duration(1).sleep();
            ROS_INFO("Wait for point_cloud_2");
//            this->reset();
//            pcl::PCDReader reader;
//            reader.read ("/home/kumamon/frank_ku/code/cluster_extraction/table_scene_lms400.pcd", *cloud);
//            writer.write<pcl::PointXYZ>(this->path.str()+"cloud_filtered__.pcd", *cloud, false);
//            this->flag = true;

        }
        object_3d_detector::Object3DsResult::Ptr result = this->compute(cloud);
//        std::cout << *result << std::endl;
        ROS_INFO("ClusterExtraction COMPLETED");
        server.setSucceeded(*result);
	this->flag = false;
    }

};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "cluster_extraction");
    ClusterExtraction clusterExtraction("/object/cluster_extraction");

    ros::spin();
    return (0);
}
