/*
 * Description : Do(and read) index using flann & surf
 * Author      : Chanon Onman
 */

#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <vector>

//
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Vector3.h>

using namespace std;
using namespace cv;
using namespace cv_bridge;

#define TOPIC_CONTROL "/cmd_state"

int gROI_x1 = 0;
int gROI_y1 = 0;
int gROI_x2 = 640;
int gROI_y2 = 480;

char *imgLibDir = "./img-lib";

char *queryFile = "query.000.bmp";
char fileName[1024];
int numSaveFrame = 0;
int numObj = 0;
float dist[480][640];
int canPrintDepth = 0;
double min_range_;
double max_range_;
int curObj = 0;

cv::Mat depthImg ;
cv_bridge::CvImagePtr bridge;
ros::Publisher vector_pub; // = n2.advertise<geometry_msgs::Vector3>("object_point", 1000);
typedef struct {
	int numObj;
	char **label;
	int *numPic;
	cv::flann::Index *index;
	cv::Mat desc_mat; // surf descriptor
	cv::Mat ind_mat;  // label(ID)
}IndexBook;
IplImage *inFrame  = cvCreateImage(cvSize(640, 480), 8, 3);

int get_dest = 0;
char obj_label[20];

void convertmsg2img(const sensor_msgs::ImageConstPtr& msg);
IndexBook* load_index(char* dirpath);

void controlCallBack(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("get command : %s\n",msg->data.c_str());
	IndexBook *indexBook = load_index(imgLibDir);
	for(int obj = 0; obj < indexBook->numObj; obj++){
		//fprintf(fp, "%s %d\n", indexBook->label[obj], indexBook->numPic[obj]);
		if(!strcmp(indexBook->label[obj],msg->data.c_str()))
		{
			get_dest = 1;
			strcpy((char*)msg->data.c_str(),obj_label);
		}
	}
}



void DepthToWorld(float * x, float * y, float depth)
{
    static const double fx_d = 1.0 / 5.9421434211923247e+02;
    static const double fy_d = 1.0 / 5.9104053696870778e+02;
    static const double cx_d = 3.3930780975300314e+02;
    static const double cy_d = 2.4273913761751615e+02;
    *x = float( (*x - cx_d) * depth * fx_d);
    *y = float( (*y - cy_d) * depth * fy_d);
}


void depthCb( const sensor_msgs::ImageConstPtr& image )
{
    canPrintDepth = 0;
    try
    {
        bridge = cv_bridge::toCvCopy(image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform depth image.");
        return;
    }
   // printf("%d %d \n", bridge->image.cols,bridge->image.rows);
    depthImg = Mat(bridge->image.rows,bridge->image.cols, CV_8UC1);
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
    canPrintDepth = 1;
}

// mouse callback
void on_mouse( int event, int x, int y, int flags, void* param )
{
	if(event == CV_EVENT_LBUTTONUP) {
		printf("%d, %d\n", x, y);
		gROI_x1 = x;
		gROI_y1 = y;
	}
	else if(event == CV_EVENT_RBUTTONUP) {
		printf("%d, %d\n", x, y);
		gROI_x2 = x;
		gROI_y2 = y;
	}
}

bool is_updated(char* dirpath)
{
	int  numObj, updated;
	char regPath[1024];
	sprintf(regPath, "%s/registry.txt", dirpath);
	FILE *fp = fopen(regPath, "r");
	fscanf(fp, "%d", &numObj);	// read number of objects
	fscanf(fp, "%d", &updated);	// read whether it is updated or not
	return updated > 0;
}

void write_updated(char* dirpath, IndexBook *indexBook)
{
	char regPath[1024];
	sprintf(regPath,"%s/registry.txt",dirpath);
	FILE *fp = fopen(regPath, "w");
	fprintf(fp, "%d 1\n", indexBook->numObj);
	for(int obj = 0; obj < indexBook->numObj; obj++){
		fprintf(fp, "%s %d\n", indexBook->label[obj], indexBook->numPic[obj]);
	}
	fclose(fp);
}

void write_edited(char* dirpath, IndexBook *indexBook)
{
	char regPath[1024];
	sprintf(regPath,"%s/registry.txt",dirpath);
	FILE *fp = fopen(regPath, "w");
	fprintf(fp, "%d 0\n", indexBook->numObj);
	for(int obj = 0; obj < indexBook->numObj; obj++){
		fprintf(fp, "%s %d\n", indexBook->label[obj], indexBook->numPic[obj]);
	}
	fclose(fp);
}

typedef enum {
	LinearIndex = 0,
	KDTreeIndex,
	KMeanTreeIndex
} IndexType;

void do_index(char* dirpath, IndexType indexType=LinearIndex)
{
	int  numObj, updated;
	char regPath[1024];			//registry.txt
	char subObjPath[1024];
	char treeFilePath[1024];	//structure file
	char contentFilePath[1024]; //content(leaves node) file
	char indexFilePath[1024];   //index for each node file

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSURFParams params   = cvSURFParams(500, 1);

	sprintf(regPath, "%s/registry.txt", dirpath);
	FILE *fp = fopen(regPath, "r");
	fscanf(fp, "%d", &numObj);	// read number of objects
	fscanf(fp, "%d", &updated);	// read whether it is updated or not

	CvSeq ***keypList = new CvSeq**[numObj];
	CvSeq ***descList = new CvSeq**[numObj];
	CvMemStorage ***storageList = new CvMemStorage**[numObj];
	int *numPic  = new int[numObj];
	char **label = new char*[numObj];
	int numDescp = 0;
	
	for(int obj = 0; obj < numObj; obj++) {
		label[obj] = new char[128];
		fscanf(fp, "%s", label[obj]);
		fscanf(fp, "%d", &numPic[obj]);

		keypList[obj]    = new CvSeq*[numPic[obj]];
		descList[obj]    = new CvSeq*[numPic[obj]];
		storageList[obj] = new CvMemStorage*[numPic[obj]];

		for(int p = 0; p < numPic[obj]; p++) {
			sprintf(subObjPath, "%s/%s/%03d.bmp", dirpath, label[obj], p);

			IplImage *img = cvLoadImage(subObjPath, CV_LOAD_IMAGE_GRAYSCALE);
			if(!img) {
				printf("[WARNING][Indexing] : skip [%s], file not found\n", subObjPath);
				continue;
			}

			keypList[obj][p] = 0;
			descList[obj][p] = 0;
			storageList[obj][p] = cvCreateMemStorage(0);
			cvExtractSURF(img, 0, &keypList[obj][p], &descList[obj][p], storageList[obj][p], params);

			numDescp += descList[obj][p]->total;
			cvReleaseImage(&img);
		}
	}
	// copy to matrix
	int length = 128; //it should be 128 == number of dimension of a surf-vector
	cv::Mat m_desc(numDescp, length, CV_32F); // descriptor
	cv::Mat m_ind(numDescp, 1, CV_32S); // index
	float* desc_ptr = m_desc.ptr<float>(0);
	int*   ind_ptr  = m_ind.ptr<int>(0);

	for(int obj = 0; obj < numObj; obj++) {
		for(int p = 0; p < numPic[obj]; p++) {
			CvSeqReader desc_reader;
			cvStartReadSeq(descList[obj][p], &desc_reader);
			for(int s = 0; s < descList[obj][p]->total; s++ ) {
				const float* descriptor = (const float*)desc_reader.ptr;
				
				CV_NEXT_SEQ_ELEM(desc_reader.seq->elem_size, desc_reader);

				memcpy(desc_ptr, descriptor, length*sizeof(float));
				memcpy(ind_ptr, &obj, sizeof(int));

				desc_ptr += length;
				ind_ptr  += 1;
			}
		}
	}

	// save tree content(exclude structure)
	sprintf(contentFilePath, "%s/object.cont", dirpath);
	sprintf(indexFilePath, "%s/index.cont", dirpath);
	CvMat temp_desc = m_desc;
	CvMat temp_ind  = m_ind;
	cvSave(contentFilePath, (CvMat*)&temp_desc);
	cvSave(indexFilePath, (CvMat*)&temp_ind);

	// save tree structure
	sprintf(treeFilePath, "%s/tree.ind", dirpath);
	cv::flann::Index *index = 0;
	if(indexType == LinearIndex) {
		index = new cv::flann::Index(m_desc, cv::flann::LinearIndexParams());  // brute force
	}
	else if(indexType == KDTreeIndex) {
		index = new cv::flann::Index(m_desc, cv::flann::KDTreeIndexParams(1));  // using 4 randomized kdtrees
	}
	else if(indexType == KMeanTreeIndex) {
		index = new cv::flann::Index(m_desc, cv::flann::KMeansIndexParams(32));  // k-mean tree
	}
	else {
		index = new cv::flann::Index(m_desc, cv::flann::AutotunedIndexParams(0.2f));  // auto
	}
	index->save(treeFilePath);

	fclose(fp);
}

IndexBook* load_index(char* dirpath)
{
	char regPath[1024];

	char treeFilePath[1024];
	char descFilePath[1024];
	char indFilePath[1024];
	int  objNumPic, updated;

	IndexBook *indexBook = new IndexBook;
	
	sprintf(regPath, "%s/registry.txt", dirpath);
	FILE *regFp = fopen(regPath, "r");

	// number of objects
	fscanf(regFp, "%d", &indexBook->numObj);
	fscanf(regFp, "%d", &updated);

	// allocate mem.
	indexBook->label  = new char*[indexBook->numObj];
	indexBook->numPic = new int[indexBook->numObj];

	// labels & num pic.
	for(int obj = 0; obj < indexBook->numObj; obj++) {
		indexBook->label[obj] = new char[128];
		fscanf(regFp, "%s", indexBook->label[obj]);
		fscanf(regFp, "%d", &indexBook->numPic[obj]);
	}

	sprintf(descFilePath, "%s/object.cont", dirpath);
	CvMat *buff_desc_mat = (CvMat*)cvLoad(descFilePath);
	indexBook->desc_mat  = cv::cvarrToMat(buff_desc_mat);

	sprintf(indFilePath, "%s/index.cont", dirpath);
	CvMat *buff_ind_mat = (CvMat*)cvLoad(indFilePath);
	indexBook->ind_mat  = cv::cvarrToMat(buff_ind_mat);

	sprintf(treeFilePath, "%s/tree.ind", dirpath);
	indexBook->index = new cv::flann::Index(indexBook->desc_mat, cv::flann::SavedIndexParams(treeFilePath));

	return indexBook;
}

/// create (new) a [#desc]x128 matrix
/// should 'delete' if don't use
cv::Mat* createSurfMat(CvSeq *surf_desc)
{
	int length = (int)(surf_desc->elem_size/sizeof(float)); //it should be 128
	cv::Mat *mat = new cv::Mat(surf_desc->total, length, CV_32F);

	CvSeqReader seqReader;
	float *matPtr = mat->ptr<float>(0);
    cvStartReadSeq( surf_desc, &seqReader );
    for(int i = 0; i < surf_desc->total; i++ )
    {
        const float* descriptor = (const float*)seqReader.ptr;
        CV_NEXT_SEQ_ELEM( seqReader.seq->elem_size, seqReader );
        memcpy(matPtr, descriptor, length*sizeof(float));
        matPtr += length;
    }
	return mat;
}

typedef struct {
	int  objID;
	int  srcID;
	int  dstID;
	float dist;
}CorrespondPoint;

// mrkImg must have 3 channels(BGR)
void findObjectAndMark(IplImage *srcImg, IplImage *mrkimg, IndexBook *indexBook, IplImage *colorImg=0)
{
	float nnRatio   = 0.36f; // default is 0.3
	IplImage *queryImg = srcImg;
	IplImage *markImg  = mrkimg;
	cvCvtColor(queryImg, markImg, CV_GRAY2RGB);

	CvScalar colors[] = 
	{
		{{0,0,255}},
		{{255,0,0}},
		{{0,255,0}},
		{{0,128,255}},
		{{0,255,255}},
		{{255,128,0}},
		{{255,255,0}},
		{{255,0,255}},
		{{255,255,255}}
	};

	// extract surf
	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSURFParams  params  = cvSURFParams(500, 1);
	CvSeq *keypoints = 0;
	CvSeq *desc      = 0;
	cvExtractSURF(queryImg, 0, &keypoints, &desc, storage, params);

	// create query matrix
	cv::Mat *queryVector = createSurfMat(desc);

	cv::Mat m_indices(queryVector->rows, 2, CV_32S);
	cv::Mat m_dists(queryVector->rows, 2, CV_32F);
	indexBook->index->knnSearch(*queryVector, m_indices, m_dists, 2, cv::flann::SearchParams(2));

	vector<CorrespondPoint> correspondList;
	int* indices_ptr = m_indices.ptr<int>(0);
	float* dists_ptr = m_dists.ptr<float>(0);
	float *sumDist =  new float[indexBook->numObj];
	int *numNN =  new int[indexBook->numObj];
	int numAll = 0;
	for(int obj = 0; obj < indexBook->numObj; obj++) {
		sumDist[obj] = 0.0f;
		numNN[obj]   = 0;
	}
	for (int i=0;i<m_indices.rows;++i) {
		// compare 1st and 2nd distance
		if (dists_ptr[2*i] < nnRatio*dists_ptr[2*i+1]) {
			// store point
			CorrespondPoint c;
			c.dstID = i;
			c.srcID = indices_ptr[2*i];
			c.dist  = dists_ptr[2*i];
			c.objID = *(indexBook->ind_mat.ptr<int>(c.srcID));
			correspondList.push_back(c);

			numAll++;
			numNN[c.objID]++;
			sumDist[c.objID] += dists_ptr[2*i];
		}
	}

	CorrespondPoint *correspond = new CorrespondPoint[numAll];
	int i = 0;
	for(vector<CorrespondPoint>::iterator it = correspondList.begin(); it != correspondList.end(); it++){
		correspond[i++] = *it;
	}


	for(int obj = 0; obj < indexBook->numObj; obj++) {

		if(numNN[obj] > 5) { // default is 5  check for have objecy in sence
			float sum_x = 0.0f;
			float sum_y = 0.0f;
			int max = -1;
			int avg_x;
			int avg_y;
			float max_dist = 1000;
			for(int p = 0; p < numAll; p++) {
				if(correspond[p].objID != obj) {
					continue;
				}
				CvSURFPoint *surf = (CvSURFPoint*)cvGetSeqElem(keypoints, correspond[p].dstID);

				int xx = surf->pt.x;
				int yy = surf->pt.y;
				unsigned char r_val = colorImg->imageData[yy*colorImg->width*colorImg->nChannels + xx*colorImg->nChannels + 2];
				unsigned char g_val = colorImg->imageData[yy*colorImg->width*colorImg->nChannels + xx*colorImg->nChannels + 1];
				unsigned char b_val = colorImg->imageData[yy*colorImg->width*colorImg->nChannels + xx*colorImg->nChannels + 0];
				int sum_group = 0;
				float r_group = 50.0f; // radius
				float max_dist_tmp = -1;
				for(int p2 = 0; p2 < numAll; p2++) {
					if(correspond[p2].objID != obj || p2 == p) {
						continue;
					}
					CvSURFPoint *surf2 = (CvSURFPoint*)cvGetSeqElem(keypoints, correspond[p2].dstID);
					
					if( sqrt( pow(xx-surf2->pt.x,2)+pow(xx-surf2->pt.x,2) ) < r_group  
						//&& dist[(int)surf2->pt.y][(int)surf2->pt.x] <= 2.0f
					)
					{
						if( sqrt( pow(xx-surf2->pt.x,2)+pow(xx-surf2->pt.x,2) ) > max_dist)
						{
							max_dist_tmp= sqrt( pow(xx-surf2->pt.x,2)+pow(xx-surf2->pt.x,2) );
						}
						sum_group++;
					}
				}

				if(sum_group > max || ( sum_group == max && max_dist > max_dist_tmp )) { 
					max_dist = max_dist_tmp;
					max = sum_group; 
					avg_x = xx;
					avg_y = yy;
				}

				//if( dist[yy][xx] <= 2.0f){
				//	sum_x += surf->pt.x;
				//	sum_y += surf->pt.y;		
					cvCircle(markImg, cvPoint(xx,yy), 1, colors[obj], 2);		
				//}
			
				
				//printf("%.1f %.1f\n",surf->pt.x,surf->pt.y);
			}
			cvCircle(markImg, cvPoint(avg_x,avg_y), 1, colors[obj], 2);

			if(max> 5){
				if(get_dest)
				{
					if(dist[avg_y][avg_x] < 2)
					{
						float tmp_z = dist[avg_y][avg_x];
						float tmp_x = avg_x;
						float tmp_y = avg_y;
						DepthToWorld(&tmp_x,&tmp_y,tmp_z);
						printf("%s  -> x:%d y:%d z:%.2f\n",indexBook->label[obj],avg_x,avg_y,dist[avg_y][avg_x]);
						printf("%s  -> x:%.2f y:%.2f z:%.2f\n",indexBook->label[obj],tmp_x,tmp_y,tmp_z);
						geometry_msgs::Vector3 vector;
						vector.x = tmp_x;
						vector.y = tmp_y;
						vector.z = tmp_z;						
						vector_pub.publish(vector);
					}
				}
				cvPutText(markImg, indexBook->label[obj], cvPoint(avg_x,avg_y), &cvFont(1.0, 2), colors[obj]);
			}
		}
	}

	delete correspond;
	delete queryVector;
}

void kinectCallBack(const sensor_msgs::ImageConstPtr& msg)
{
	int inKey = 0;
	bool editLib = false;
	IplImage *grayImg  = cvCreateImage(cvSize(640, 480), 8, 1);	
	IplImage *markImg  = cvCreateImage(cvSize(640, 480), 8, 3);
	
//if(canPrintDepth) cv::imshow("win2",depthImg);
	IndexBook *indexBook = load_index(imgLibDir);

	for(int i=0;i<640*480;i++)
	{
		
		if(dist[i/640][i%640] < 2 || 1 )
		{
			//printf("%d %d %.2f\n",i/480,i%480,dist[i/480][i%480]);
			inFrame->imageData[i*3] = msg->data[i*3+2];
			inFrame->imageData[i*3+1] = msg->data[i*3+1];
			inFrame->imageData[i*3+2] = msg->data[i*3];
		}
		else
		{
			inFrame->imageData[i*3] = 255;
			inFrame->imageData[i*3+1] = 0;
			inFrame->imageData[i*3+2] = 255;
		}	
	}		
	
	//convertmsg2img(msg);
	cvCvtColor(inFrame, grayImg, CV_BGR2GRAY);
	//cvCvtColor(grayImg, markImg, CV_GRAY2RGB);
	findObjectAndMark(grayImg, markImg, indexBook, inFrame);
	//output
	cvDrawRect(markImg, cvPoint(gROI_x1,gROI_y1), cvPoint(gROI_x2,gROI_y2), CV_RGB(0,255,0));
	cvPutText(markImg, indexBook->label[curObj], cvPoint(gROI_x1,gROI_y1), &cvFont(1.5,2), CV_RGB(255,255,255));

	cvShowImage("input", markImg);

	inKey = cvWaitKey(1);
	if(inKey == 27){
		exit(0);
	}
	else if(inKey == 32){
		printf("fetch\n");
		int picID = indexBook->numPic[curObj];
		char *picName = indexBook->label[curObj];
		cvSetImageROI(inFrame, cvRect(gROI_x1, gROI_y1, gROI_x2 - gROI_x1, gROI_y2 - gROI_y1));
		
		sprintf(fileName,"%s/%s/%03d.bmp",imgLibDir,picName,picID);
		cvSaveImage(fileName, inFrame);
		cvResetImageROI(inFrame);
		editLib = true;

		sprintf(fileName,"gray-%03d.bmp",numSaveFrame);
		cvSaveImage(fileName, markImg);
		
		indexBook->numPic[curObj]++;
		numSaveFrame++;
	}
	else if(inKey == 2555904){
		curObj = (curObj+1)%indexBook->numObj;
	}
	else if(inKey == 2424832){
		curObj = (curObj+(indexBook->numObj-1))%indexBook->numObj;
	}
	else if(inKey == 'a'){
		get_dest = 1;
	}
	else if(inKey >= 0){
		printf("key = %d\n", inKey);
	}

	if(editLib) {
		write_edited(imgLibDir, indexBook);
	}

	cvReleaseImage(&grayImg);
	cvReleaseImage(&markImg);

}

int main(int argc , char *argv[])
{

	float nnRatio   = 0.3f;
	bool reindexing = true;
//	bool camera_running = true;
	CvCapture* capture = 0;
	int frameWidth  = 0;
	int frameHeight = 0;
	
	  
	IndexBook *indexBook = load_index(imgLibDir);

	if(reindexing || !is_updated(imgLibDir)) {
		printf("[Initialize] : reindexing\n");
		do_index(imgLibDir, KDTreeIndex);
	}

	printf("[Initialize] : reading index\n");
	write_updated(imgLibDir, indexBook);

	ros::init(argc,argv,"objects");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	
	nh.param("min_range", min_range_, 0.5);
	nh.param("max_range", max_range_, 5.5);

	ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,kinectCallBack);
	ros::Subscriber subDepth = n.subscribe("/camera/depth/image",1,depthCb);
	ros::Subscriber sub2 = n.subscribe(TOPIC_CONTROL, 1, controlCallBack);
	vector_pub = n.advertise<geometry_msgs::Vector3>("object_point", 1000);
	

	printf("ros : spin\n");
	cvNamedWindow("input", 1 );
	cvSetMouseCallback("input", on_mouse);
	ros::spin();

}
void convertmsg2img(const sensor_msgs::ImageConstPtr& msg)
{
	for(int i=0;i<640*480;i++)
	{
		inFrame->imageData[i*3] = msg->data[i*3+2];
		inFrame->imageData[i*3+1] = msg->data[i*3+1];
		inFrame->imageData[i*3+2] = msg->data[i*3];	
	}				
}
