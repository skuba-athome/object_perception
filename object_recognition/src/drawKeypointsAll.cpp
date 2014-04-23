#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/legacy/compat.hpp>

using namespace cv;
using namespace std;

void readme();

/** @function main */
int main( int argc, char** argv )
{
    //if( argc != 3 )
//    if( argc != 2 )
//    { readme(); return -1; }

    CvMemStorage* storage = cvCreateMemStorage(0);
    CvMat* prevgray = 0, *image = 0, *gray =0;
    int key = 0;
    static CvScalar red_color[] ={0,0,255};
    cout << argc << endl;
    for(int j=0;j<38;j++){
        //std::stringstream ss;
        char fileName[100],output[100];
        //sprintf(fileName,"/home/skuba/skuba_athome/object_perception/data/axe/frame%04d.png",j);
        sprintf(fileName,"/home/skuba/skuba_athome/object_perception/data/%s/frame%04d.png",argv[1],j);
        //sprintf(fileName,"/home/skuba/skuba_athome/object_perception/data/%s/miniteMaid_%d.jpg",argv[1],j);
        sprintf(output,"/home/skuba/skuba_athome/object_perception/tmp/frame%04d_feature.png",j);

        cout << fileName << endl;

        Mat img_1 = imread( fileName, CV_LOAD_IMAGE_GRAYSCALE );

        if( !img_1.data)// || !img_2.data )
        { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

        IplImage* iplImage = new IplImage(img_1);


        CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
        int i;

        //Extract SURF points by initializing parameters
        CvSURFParams params = cvSURFParams(500, 1);
        //cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );

        cvExtractSURF( iplImage, 0, &imageKeypoints, &imageDescriptors, storage, params );
        printf("Image Descriptors: %d\n", imageDescriptors->total);




        //-- Step 1: Detect the keypoints using SURF Detector
//        int minHessian = 400;
//
//        SurfFeatureDetector detector( minHessian );
//
//        std::vector<KeyPoint> keypoints_1, keypoints_2;
//
//        detector.detect( img_1, keypoints_1 );
        //detector.detect( img_2, keypoints_2 );

        //-- Draw keypoints
        Mat img_keypoints_1; Mat img_keypoints_2;

        //        drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


        for( i = 0; i < imageKeypoints->total; i++ )
        {
            CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
            CvPoint center;
            int radius;
            center.x = cvRound(r->pt.x);
            center.y = cvRound(r->pt.y);
            radius = cvRound(r->size*1.2/9.*2);
            cvCircle( iplImage, center, radius, red_color[0], 1, 8, 0 );
        }

        //drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


        //-- Show detected (drawn) keypoints
        //imshow("Keypoints 1", img_keypoints_1 );
        //imshow("Keypoints 2", img_keypoints_2 );

        //        namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
        //        imshow( "Display window", img_keypoints_1 );                   // Show our image inside it.


        vector<int> compression_params; //vector that stores the compression parameters of the image
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //specify the compression technique
        //compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
        //compression_params.push_back(100); //specify the compression quality
        compression_params.push_back(9); //specify the compression quality
        std::stringstream ss;
        //bool bSuccess = imwrite(output, cv::Mat(img_keypoints_1), compression_params); //write the image to file
        bool bSuccess = imwrite(output, cv::Mat(iplImage), compression_params); //write the image to file
    }

    return 0;
}

/** @function readme */
void readme()
{ std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl; }
