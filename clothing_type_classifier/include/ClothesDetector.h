//
// Created by kandithws on 28/2/2559.
//

#ifndef CLOTHES_DETECTOR_CLOTHES_DETECTOR_H
#define CLOTHES_DETECTOR_CLOTHES_DETECTOR_H

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "egbis.h"

class ClothesDetector
{
    public:
        ClothesDetector();
        void findSegmentBoundary();
        void detectClothesObjects();
        void getBinaryImage(std::vector<cv::Mat>& images, std::vector<cv::Mat>& image_th, int threshold_value);


    private:

};


#endif //CLOTHES_DETECTOR_CLOTHES_DETECTOR_H
