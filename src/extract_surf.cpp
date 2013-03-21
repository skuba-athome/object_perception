#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

using namespace std;
using namespace cv;

void readme();

int main(int argc, char** argv) {
	if( argc != 3 ) { readme(); return -1; }
	
	Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	if( !img_object.data )
 	{ cout << " --(!) Error reading images " << endl; return -1; }
	
	int minHessian = 400;

	SurfFeatureDetector detector( minHessian );
	std::vector<KeyPoint> keypoints_object;
	detector.detect( img_object, keypoints_object );
	//cout << "Size of keypoint : " <<keypoints_object.size() << endl;
	
	SurfDescriptorExtractor extractor;
	Mat descriptors_object;
	extractor.compute( img_object, keypoints_object, descriptors_object );
	//cout << "rows : " << descriptors_object.rows << " cols : " << descriptors_object.cols << endl;
	for(register int i = 0; i < descriptors_object.rows; ++i) {
		cout << argv[2] << " ";
    		for(register int j = 0; j < descriptors_object.cols; ++j) {
        		float Pixel = descriptors_object.at<float>(i, j);
			cout <<  j+1<<":"<<Pixel << " ";
    		}
		cout << endl;
	}
	//cout << descriptors_object << endl;
	
	return 0;
}

/** @function readme */
void readme(){ 
	std::cout << " Usage: ./extract_surf <img1> <label>" << std::endl; 
}

