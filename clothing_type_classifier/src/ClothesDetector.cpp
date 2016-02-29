//
// Created by kandithws on 28/2/2559.
//

#include "ClothesDetector.h"

using namespace cv;

ClothesDetector::ClothesDetector()
{

}


void ClothesDetector::getBinaryImage(std::vector<cv::Mat>& images, std::vector<cv::Mat>& image_th ,int threshold_value)
{
    std::vector<cv::Mat> images_gray(images.size());
    image_th.clear();
    image_th.resize(images.size());

    for(int i = 0; i< images.size(); i++)
    {
        cvtColor( images[i], images_gray[i], CV_BGR2GRAY );
        threshold( images_gray[i], image_th[i], threshold_value, 255, THRESH_BINARY);
    }

}

