//
// Created by kandithws on 28/2/2559.
//

#include <ClothesDetector.h>


using namespace cv;


ClothesDetector::ClothesDetector()
{

}

ClothesDetector::DetectorDescriptors::DetectorDescriptors()
{

}

void ClothesDetector::setOriginalImage(cv::Mat img)
{
    this->original = img;
}

void ClothesDetector::DetectorDescriptors::copyTo(DetectorDescriptors &target)
{
    this->contour_area = target.contour_area;
    this->centroid = target.centroid;
    this->contour.clear();
    this->contour.insert(this->contour.begin(), target.contour.begin(), target.contour.end());
    this->mask = cv::Mat::zeros(target.mask.rows, target.mask.cols, target.mask.type());
    if(!target.mask.empty())
        target.mask.copyTo(this->mask);
    if(!target.cropped.empty())
       target.cropped.copyTo(this->cropped);
    this->rect = target.rect;
    this->type = target.type;
}

inline void ClothesDetector::DetectorDescriptors::operator = ( DetectorDescriptors &target )
{
    this->contour_area = target.contour_area;
    this->centroid = target.centroid;
    this->contour.clear();
    this->contour.insert(this->contour.begin(), target.contour.begin(), target.contour.end());
    if(!target.mask.empty())
        this->mask = cv::Mat(target.mask);
    if(!target.cropped.empty())
        this->cropped = cv::Mat(target.cropped);
    this->rect = target.rect;
    this->type = target.type;
}



void ClothesDetector::detectClothesObjects(std::vector<cv::Mat> &images_th, std::vector<DetectorDescriptors>& out, bool crop_original)
{
    int max_rect_area = 0;
    int idx = 0;
    if(!out.empty())
    {
        std::cout << "ClothesDetector::detectClothesObjects => Output vector is not empty" << std::endl;
        return;
    }
    out.resize(images_th.size());
    for(int i = 0; i < images_th.size(); i++)
    {

        DetectorDescriptors  temp_desc;
        cv::Mat temp_mat;
        images_th[i].copyTo(temp_mat);
        this->computeDescriptors(temp_mat, temp_desc);
        int area = temp_desc.rect.area();
        images_th[i].copyTo(temp_desc.mask);
        if(area > max_rect_area)
        {
            max_rect_area = area;
            idx = i;
        }
        out[i] = temp_desc;
    }
    for(int i=0 ; i < out.size(); i++)
    {
        if( i == idx)
            out[i].type = TABLE;
        else
            out[i].type = CLOTHES;
    }
    if(crop_original)
    {
        this->cropOriginal(out);
    }

}


void ClothesDetector::saveOutputImages(std::vector<DetectorDescriptors>& images, std::string filename,
                                       bool draw_descriptors)
{
    for(int i = 0; i < images.size(); i++)
    {
        std::string temp;
        temp = filename + '_' + std::to_string(i) + ".jpg";
        imwrite(temp, images[i].cropped);
        if(draw_descriptors)
        {
            cv::Mat drawing;
            this->drawDescriptors(images[i], drawing);
            temp.clear();
            temp = filename + '_' + std::to_string(i) + "_desc.jpg";
            imwrite(temp, drawing);
        }

    }
}


void ClothesDetector::cropOriginal(std::vector<DetectorDescriptors>& out)
{
    if(this->original.empty())
    {
        std::cout << "Abort ClothesDetector::cropOriginal :: the original image has not been set." << std::endl;
    }
    for(int i=0 ; i < out.size(); i++)
    {
        cv::Mat temp;
        this->original.copyTo(temp, out[i].mask);
        out[i].cropped = cv::Mat(temp, out[i].rect);
    }
}

void ClothesDetector::getBinaryImage(std::vector<cv::Mat>& images, std::vector<cv::Mat>& image_th, int threshold_value,
                                     int closing_window_size, int opening_window_size, int kernel_type)
{
    if(!image_th.empty())
    {
        std::cout << " ClothesDetector::getBinaryImage => Output vector is not empty" << std::endl;
        return;
    }
    std::vector<cv::Mat> images_gray(images.size());
    const int OPERATION_OPENING = 2;
    const int OPERATION_CLOSING = 3;
    image_th.resize(images.size());


    //Morphological Kernel MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE
    Mat closing_element = getStructuringElement( kernel_type,
                                         Size( 2*closing_window_size+ 1, 2*closing_window_size+1 ),
                                         Point( closing_window_size, closing_window_size ) );

    Mat opening_element = getStructuringElement( kernel_type,
                                         Size( 2*opening_window_size + 1, 2*opening_window_size+1 ),
                                         Point( opening_window_size, opening_window_size ) );

    for(int i = 0; i< images.size(); i++)
    {
        try
        {
            cvtColor( images[i], images_gray[i], CV_BGR2GRAY );
            threshold( images_gray[i], image_th[i], threshold_value, 255, THRESH_BINARY);
            morphologyEx(image_th[i], image_th[i], OPERATION_CLOSING, closing_element);
            morphologyEx(image_th[i], image_th[i], OPERATION_OPENING, opening_element);
        }
        catch ( cv::Exception & e )
        {
            std::cout << e.what() << std::endl;
            exit(0);
        }
    }
}



void ClothesDetector::computeDescriptors(cv::Mat images_th, DetectorDescriptors &out)
{
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    /// Find contours
    findContours( images_th, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    /// Approximate contours to polygons + get bounding rects
    std::vector<std::vector<Point> > contours_poly( contours.size() );
    std::vector<Rect> boundRect( contours.size() );

    int max_area_index = 0;
    int max_area = 0;

    if(contours.empty())
    {
        std::cout << "Cannot Detect Contour: Abort Computing Descriptors" << std::endl;
        return;
    }

    for( int j = 0; j < contours.size(); j++ )
    {
        approxPolyDP( Mat(contours[j]), contours_poly[j], 3, true );
        boundRect[j] = boundingRect( Mat(contours_poly[j]) );
        //Get the contour which has maximum rect area
        int area = boundRect[j].area();
        if(area > max_area)
        {
            max_area = area;
            max_area_index = j;
        }
    }

    out.rect = boundRect[max_area_index];
    out.contour = contours[max_area_index];
    out.contour_area = contourArea(contours[max_area_index]);
    out.type = UNKNOWN;

    //Compute Centroid (Central Moments of Shape)
    Moments mu= moments( contours[max_area_index], false );
    out.centroid = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

}

void ClothesDetector::drawDescriptors(DetectorDescriptors& input, cv::Mat& output)
{
    this->original.copyTo(output);
    std::vector<std::vector<Point>> contours_poly(1);
    approxPolyDP( Mat(input.contour), contours_poly[0], 3, true );
    Scalar color = Scalar( 0, 0, 255);
    circle(output, input.centroid, 3, color,4); //Marking Centroid of main segment
    drawContours( output, contours_poly, 0, color, 1, 8, std::vector<Vec4i>(), 0, Point() );
    rectangle( output, input.rect.tl(), input.rect.br(), color, 2, 8, 0 );
    std::string print = "Type: ";
    if(input.type == TABLE)
        print = print + "Table";
    else if(input.type == CLOTHES)
        print = print + "Clothes";
    else
        print = print + "Unknown";
    putText(output, print, Point(0,25), FONT_HERSHEY_PLAIN, 2, color, 3, LINE_8);
}


