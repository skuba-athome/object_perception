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

        class DetectorDescriptors
        {
            public:
                DetectorDescriptors();
                std::vector<cv::Point> contour;
                double contour_area;
                cv::Point2f centroid;
                cv::Rect rect;
                cv::Mat mask;
                cv::Mat cropped;
                uint8_t type;
                void copyTo(DetectorDescriptors &target);
                inline void operator = ( DetectorDescriptors &target );
        };
        enum
        {
                TABLE,
                CLOTHES,
                UNKNOWN
        };

        ClothesDetector();
        //Morphological Kernel MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE
        void setOriginalImage(cv::Mat img);
        void getBinaryImage(std::vector<cv::Mat>& images, std::vector<cv::Mat>& image_th, int threshold_value,
                            int closing_window_size = 5, int opening_window_size = 5, int kernel_type = cv::MORPH_ELLIPSE );

        void detectClothesObjects(std::vector<cv::Mat> &images_th, std::vector<DetectorDescriptors>& out, bool crop_original = true);
        void saveOutputImages(std::vector<DetectorDescriptors>& images, std::string filename = "out",
                              bool draw_descriptors = true);

    private:
        cv::Mat original;
        void computeDescriptors(cv::Mat images_th, DetectorDescriptors &out);
        void drawDescriptors(DetectorDescriptors& input, cv::Mat& output);
        void cropOriginal(std::vector<DetectorDescriptors>& out);
};


#endif //CLOTHES_DETECTOR_CLOTHES_DETECTOR_H
