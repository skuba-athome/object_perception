#include <stdio.h>
#include <string>
#include <dirent.h>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "object_perception/verifyObject.h"
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"

using namespace cv;
using namespace std;

typedef struct ObjectModel {
    string objectName;
    string configPath;
} ObjectModel;

Mat centroids;
vector<ObjectModel> objectModelContainer;
CvSVM svmModel;

inline float getDistance(Mat a, Mat b)
{
    float sum = 0.0f;
    for(int i=0;i < a.cols; ++i)
    {
        sum += pow(a.at<float>(0,i)-b.at<float>(0,i),2);
    }
    return sum;
}

int classifyFeature(Mat feature)
{
    float minDist = 99999.0f;
    int index = -1;
    for(int clusterNo = 0;clusterNo < centroids.rows; ++clusterNo)
    {
        float dist = getDistance(feature, centroids.row(clusterNo));
        if(dist <= minDist)
        {
            minDist = dist;
            index = clusterNo;
        }
    }
    return index;
}

Mat createHistogram(Mat features)
{
    Mat histogram(1, centroids.rows, CV_32FC1);
    for(int attributeNo=0;attributeNo < histogram.cols; ++attributeNo)
    {
        histogram.at<float>(0,attributeNo) = 0.0;
    }

    for(int index=0;index < features.rows;++index)
    {
        int attributeNo = classifyFeature(features.row(index));
        histogram.at<float>(0,attributeNo) += 1.0f;
    }
    return histogram;
}

int verifyResult(float *predictResult)
{
    int count = 0;
    int returnIndex = -1;
    for(int index=0;index < objectModelContainer.size(); ++index)
    {
        if(predictResult[index] > 0)
        {
            count++;
            returnIndex = index;
        }
    }
    if(count != 1)
    {
        return -1;
    }
    else
    {
        return returnIndex;
    }
}

bool verifyObjectService(object_perception::verifyObject::Request &req,
                         object_perception::verifyObject::Response &res)
{
    Mat image = imread(req.objectPictureFilePath, CV_LOAD_IMAGE_GRAYSCALE);
    if(image.empty())
    {
        ROS_ERROR("Can't read image.");
        return false;
    }

    // detecting keypoints
    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints;
    detector.detect(image, keypoints);

    // computing descriptors
    SurfDescriptorExtractor extractor(4,2,true);
    Mat descriptors;
    extractor.compute(image, keypoints, descriptors);

    // create histogram
    Mat unknownHistogram = createHistogram(descriptors);

    // predict
    float classResponse[objectModelContainer.size()];
    for(int objectIndex=0;objectIndex < objectModelContainer.size(); ++objectIndex)
    {
        svmModel.load(objectModelContainer[objectIndex].configPath.data());
        classResponse[objectIndex] = svmModel.predict(unknownHistogram);
    }

    // verify response
    int objectIndex = verifyResult(classResponse);
    
    if(objectIndex == -1)
    {
        res.objectName = "unknown";
    }
    else
    {
        res.objectName = objectModelContainer[objectIndex].objectName;
    }
    return true;
}

void readClusterFile(string clusterFile)
{
    FILE *ptr = fopen(clusterFile.data(), "r");
    if(ptr == NULL)
    {
        ROS_ERROR("Cannot read cluster file.");
        return;
    }
    int clusterCount;
    fscanf(ptr, "%d", &clusterCount);
    cout << "Number of attribute : " << clusterCount << endl;

    Mat centroidData(clusterCount, 128, CV_32FC1);

    for(int i=0;i<clusterCount;++i)
    {
        for(int j=0;j<128;++j)
        {
            float temp;
            fscanf(ptr, "%f", &temp);
            centroidData.at<float>(i,j) = temp;
        }
    }
    fclose(ptr);

    centroids = centroidData;
}

void loadSVMConfigs(string svmModelDir)
{
    DIR *dir = opendir(svmModelDir.data());
    if(dir == NULL)
    {
        ROS_ERROR("Cannot open svm model directory.");
        return;
    }

    struct dirent *entry;
    while((entry = readdir(dir)) != NULL)
    {
        string filename = entry->d_name;
        size_t found = filename.find(".dat");
        if(found == string::npos) continue;
        ObjectModel objectModel;
        objectModel.objectName = filename.substr(0,found);
        objectModel.configPath = svmModelDir + filename;

        cout << "Object model " << objectModel.objectName << " found." << endl;
        objectModelContainer.push_back(objectModel);
    }
    closedir(dir);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "verify_object_service");
    ros::NodeHandle n("~");

    string clusterFile;
    n.param("cluster_file", clusterFile, string("cluster.config"));
    readClusterFile(clusterFile);

    string svmModelDir;
    n.param("svm_model_dir", svmModelDir, string("svm_configs/"));
    loadSVMConfigs(svmModelDir);
    
    ros::ServiceServer service = n.advertiseService("/verifyObject", verifyObjectService);
    ROS_INFO("verify_object_service start.");
    ros::spin();

    return 0;
}
