#include "main.h"

void currnetFrame_mouse_callback( int event, int x, int y, int flags, void* param ){
	switch( event ){
		case CV_EVENT_LBUTTONDOWN :
			min_x = x;
			min_y =y;
			break;
		case CV_EVENT_RBUTTONDOWN :
			max_x = x;
			max_y = y;
			break;
	}
	//cvRectangle(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int lineType=8, int shift=0)
}

void depthCb( const sensor_msgs::ImageConstPtr& image )
{
	canUseDist = 0;
    try
    {
        bridge = cv_bridge::toCvCopy(image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform depth image.");
        return;
    }
    depthImg = Mat(bridge->image.rows, bridge->image.cols, CV_8UC1);
    for(int i = 0; i < bridge->image.rows; i++)
    {
        float* Di = bridge->image.ptr<float>(i);
        char* Ii = depthImg.ptr<char>(i);
        for(int j = 0; j < bridge->image.cols; j++)
        {
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
            dist[i][j] = Di[j];
        }
    }
    canUseDist = 1;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	IpVec ipts , ipts2 ;
	IpPairVec matches;

	for(int i=0;i<WIDTH*HIEGHT;i++)
	{
		currnetFrame->imageData[i] = msg->data[i];
	}

	//cvSmooth(currnetFrame,currnetFrame,CV_GAUSSIAN);
	cvRectangle(currnetFrame,cvPoint(min_x,min_y),cvPoint(max_x,max_y), cvScalarAll(7.0) );
	surfDetDes(currnetFrame, ipts, false, 3, 4, 3, 0.004f); // Detect and describe interest points in the frame
	IplImage *tmp = cvLoadImage("./imgs/02.pgm",CV_LOAD_IMAGE_GRAYSCALE);
	surfDetDes(tmp, ipts2, false, 3, 4, 3, 0.004f); // Detect and describe interest points in the frame
	for(int i = 0 ; i< ipts.size();i++)
	{
		float x = ipts[i].x;
		float y = ipts[i].y;

		if( dist[ (int)(y/HIEGHT*480) ][ (int)(x/WIDTH*640) ] < 2.0f )
		{
			ipts.erase(ipts.begin()+i);
			i--;
		}
		printf("%d %d %.2f\n",(int)(y/HIEGHT*480),(int)(x/WIDTH*640),dist[ (int)(y/HIEGHT*480) ][ (int)(x/WIDTH*640) ]);
	}
	printf("-------------------------------------------");
	getMatches(ipts,ipts2,matches); // Fill match vector

    for (unsigned int i = 0; i < matches.size(); i++)
    {
      const float & dx = matches[i].first.dx;
      const float & dy = matches[i].first.dy;
      if(canUseDist)// if( dist[ (int)(y/HIEGHT*480) ][ (int)(x/WIDTH*640) ] < 2.0f )
		  drawIpoint(currnetFrame , matches[i].first, 3);
    }

	// Display image
	cvShowImage("currnetFrame",currnetFrame);
	cvShowImage("tmp",tmp);
	char inKey = cvWaitKey(1);
	if(inKey == 27){
		exit(0);
	}

	cvReleaseImage(&tmp);
}


int main(int argc,char * argv[])
{
	ros::init(argc,argv,"capture");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	nh.param("min_range", min_range_, 0.5);
	nh.param("max_range", max_range_, 5.5);
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_mono",1, imageCb);
	ros::Subscriber sub2 = nh.subscribe("/camera/depth/image",1, depthCb);

	cvNamedWindow( "currnetFrame" );

	// Set up the callback
	cvSetMouseCallback( "currnetFrame", currnetFrame_mouse_callback );


	ros::spin();
}
