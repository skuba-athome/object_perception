#include <stdio.h>
#include <string>
#include "ros/ros.h"
#include "object_perception/verifyObject.h"
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"

using namespace cv;
using namespace std;

CvKNearest knnModel;

bool verifyObject(object_perception::verifyObject::Request &req,
                  object_perception::verifyObject::Response &res)
{
    Mat img1 = imread(req.objectPictureFilePath, CV_LOAD_IMAGE_GRAYSCALE);
    if(img1.empty())
    {
        ROS_ERROR("Can't read image.");
        return false;
    }

    // detecting keypoints
    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints1;
    detector.detect(img1, keypoints1);

    // computing descriptors
    SurfDescriptorExtractor extractor(4,4,true);
    Mat descriptors1;
    extractor.compute(img1, keypoints1, descriptors1);


    return true;
}

void readClusterFile(string clusterFile)
{
    FILE *ptr = fopen(clusterFile.data(), "r");
    int clusterCount;
    fscanf(ptr, "%d", &clusterCount);
    cout << "Number of attribute : " << clusterCount << endl;

    Mat trainData(clusterCount, 128, CV_32FC1);
    Mat trainLabel(clusterCount, 1, CV_32FC1);

    for(int i=0;i<clusterCount;++i)
    {
        for(int j=0;j<128;++j)
        {
            float temp;
            fscanf(ptr, "%f", &temp);
            trainData.at<float>(i,j) = temp;
        }
        trainLabel.at<float>(i,0) = i+1;
    }

    knnModel = CvKNearest(trainData, trainLabel, Mat(), false, 1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "verify_object_service");
    ros::NodeHandle n("~");

    string clusterFile;
    n.param("cluster_file", clusterFile, string("cluster.config"));
    readClusterFile(clusterFile);

    ros::ServiceServer service = n.advertiseService("verifyObject", verifyObject);
    ROS_INFO("verify_object_service start.");
    ros::spin();

    return 0;
}
