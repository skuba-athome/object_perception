#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

static void help()
{
    printf("\nThis program extract SURF keypoints\n"
            "Using the SURF desriptor:\n"
            "\n"
            "Usage:\n extractSurf <image> <file>\n");
}

int main(int argc, char** argv)
{
    //printf("argc : %d\n",argc);
//    help();

	printf("%s %s\n",argv[1],argv[2]);

    Mat img1 = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    if(img1.empty())
    {
        printf("Can't read image\n");
        return -1;
    }

    // detecting keypoints
    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints1;
    detector.detect(img1, keypoints1);

    // computing descriptors
    SurfDescriptorExtractor extractor;
    Mat descriptors1;
    extractor.compute(img1, keypoints1, descriptors1);

    //FILE *ptr = fopen(argv[2],"w");
    FILE *ptr = fopen(argv[2],"a");
    for(int i=0;i<descriptors1.rows;++i)
    {
        //fprintf(ptr,"%d,%d,",(int)keypoints1[i].pt.x,(int)keypoints1[i].pt.y);
        for(int j=0;j<descriptors1.cols;++j)
        {
            if(j != 0) fprintf(ptr," ");
	    	fprintf(ptr,"%.17lf",descriptors1.at<float>(i,j));
	}
        fprintf(ptr,"\n");
    }
    fclose(ptr);
    cout << descriptors1.rows << endl;
    return 0;
}
