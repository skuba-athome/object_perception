//*******************surf.cpp******************//
//********** SURF implementation in OpenCV*****//
//**loads video from webcam, grabs frames computes SURF keypoints and descriptors**//  //** and marks them**//

//****author: achu_wilson@rediffmail.com****//

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/legacy/compat.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

	int sum=0;
	cout << argc << endl;
	for(int i=0;i<argc;i++)
		cout << "i : " << i << " " << argv[i] << endl;
    for(int k=0;k<=41;k++){

        char fileName[100],output[100];

		if(argc == 2)
			if(strcmp(argv[1],"-s") == 0){
				sprintf(fileName,"/run/shm/object_perception/picture%d.png",k);
			}
		else
			sprintf(fileName,"/home/skuba/webcam_data_640x480/cropped/%s/frame%04d.png",argv[1],k);
			//sprintf(fileName,"/home/skuba/skuba_athome/object_perception/data/%s/frame%04d.png",argv[1],k);
        sprintf(output,"/home/skuba/skuba_athome/object_perception/object_recognition//tmp/frame%04d_feature.png",k);

		//cout << fileName << endl;

        CvMemStorage* storage = cvCreateMemStorage(0);
        cvNamedWindow("Image", 1);
        int key = 0;
        static CvScalar red_color[] ={0,0,255};
        //CvCapture* capture = cvCreateCameraCapture(0);
        CvMat* prevgray = 0, *image = 0, *gray =0;

        int firstFrame = gray == 0;
        IplImage* frame;


        Mat img = imread(fileName);//, CV_LOAD_IMAGE_GRAYSCALE );
		//cout << "img.empty() : " << img.empty() << endl;
		if(img.empty())
			continue;
        frame = new IplImage(img);

        //    cvShowImage( "Image", frame );
        //    cvWaitKey(0);

        if(!gray)
        {
            image = cvCreateMat(frame->height, frame->width, CV_8UC1);
        }

        //Convert the RGB image obtained from camera into Grayscale
        cvCvtColor(frame, image, CV_BGR2GRAY);

        //Define sequence for storing surf keypoints and descriptors
        CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
        int i;

        //Extract SURF points by initializing parameters
        CvSURFParams params = cvSURFParams(400, 1);
        //cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );

        cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
        printf("path : %s Image Descriptors: %d\n",fileName,imageDescriptors->total);
		sum+=imageDescriptors->total;

//---------------------------------------------------------
        int minHessian = 400;

        SurfFeatureDetector detector( minHessian );

        std::vector<KeyPoint> keypoints_1, keypoints_2;

        detector.detect( image, keypoints_1 );
//---------------------------------------------------------


        //draw the keypoints on the captured frame
        for( i = 0; i < imageKeypoints->total; i++ )
            //for( i = 0; i < keypoints_1.size(); i++ )
        {
            CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
            CvPoint center;
            int radius;
            center.x = cvRound(r->pt.x);
            center.y = cvRound(r->pt.y);
            radius = cvRound(r->size*1.2/9.*2);
            //cvCircle( frame, center, radius, red_color[0], 1, 8, 0 );
            cvCircle( image, center, radius, red_color[0], 1, 8, 0 );
        }
//        cvShowImage( "Image", frame );
//        cvWaitKey(0);

        vector<int> compression_params; //vector that stores the compression parameters of the image

        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //specify the compression technique
        compression_params.push_back(9); //specify the compression quality
        //bool bSuccess = imwrite(output, cv::Mat(frame), compression_params); //write the image to file
        bool bSuccess = imwrite(output, cv::Mat(image), compression_params); //write the image to file

        cvDestroyWindow("Image");
    }

	printf("sum number of descriptor: %d\n",sum);
	printf("avg number of descriptor: %f\n",sum/38.0);
//
//    CvMemStorage* storage = cvCreateMemStorage(0);
//    cvNamedWindow("Image", 1);
//    int key = 0;
//    static CvScalar red_color[] ={0,0,255};
//    //CvCapture* capture = cvCreateCameraCapture(0);
//    CvMat* prevgray = 0, *image = 0, *gray =0;
//
//    int firstFrame = gray == 0;
//    IplImage* frame;
//
//    cout << argv[1] << endl;
//
//    Mat img = imread( argv[1]);//, CV_LOAD_IMAGE_GRAYSCALE );
//    frame = new IplImage(img);
//
////    cvShowImage( "Image", frame );
////    cvWaitKey(0);
//
//    if(!gray)
//    {
//        image = cvCreateMat(frame->height, frame->width, CV_8UC1);
//    }
//
//    //Convert the RGB image obtained from camera into Grayscale
//    cvCvtColor(frame, image, CV_BGR2GRAY);
//
//    //Define sequence for storing surf keypoints and descriptors
//    CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
//    int i;
//
//    //Extract SURF points by initializing parameters
//    CvSURFParams params = cvSURFParams(500, 1);
//    //cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
//
//    cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
//    printf("Image Descriptors: %d\n", imageDescriptors->total);
//
//
//    int minHessian = 400;
//
//    SurfFeatureDetector detector( minHessian );
//
//    std::vector<KeyPoint> keypoints_1, keypoints_2;
//
//    detector.detect( image, keypoints_1 );
//
//
//    //draw the keypoints on the captured frame
//    for( i = 0; i < imageKeypoints->total; i++ )
//    //for( i = 0; i < keypoints_1.size(); i++ )
//    {
//        CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
//        CvPoint center;
//        int radius;
//        center.x = cvRound(r->pt.x);
//        center.y = cvRound(r->pt.y);
//        radius = cvRound(r->size*1.2/9.*2);
//        cvCircle( frame, center, radius, red_color[0], 1, 8, 0 );
//    }
//    cvShowImage( "Image", frame );
//    cvWaitKey(0);
//    cvDestroyWindow("Image");
    return 0;
} 
