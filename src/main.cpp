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

#include <iostream>
#include <vector>

using namespace std;

int gROI_x1 = 0;
int gROI_y1 = 0;
int gROI_x2 = 640;
int gROI_y2 = 480;

typedef struct {
	int numObj;
	char **label;
	int *numPic;
	cv::flann::Index *index;
	cv::Mat desc_mat; // surf descriptor
	cv::Mat ind_mat;  // label(ID)
}IndexBook;

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
	float nnRatio   = 0.3f;
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
	//for (int i=0;i<m_indices.rows;++i) {
	//	if (dists_ptr[10*i] < 0.6*dists_ptr[10*i+9]) {
	//		int *objNNCount = new int[indexBook->numObj];
	//		int maxCount = -1;
	//		int maxObj   = -1;
	//		for(int o = 0; o < indexBook->numObj; o++){
	//			objNNCount[o] = 0;
	//		}
	//		for(int p = 0; p < 10; p++){
	//			if(dists_ptr[10*i + p] < 0.3) {
	//				int objID = *(indexBook->ind_mat.ptr<int>(indices_ptr[10*i + p]));
	//				printf("%f\n", dists_ptr[10*i + p]);
	//				objNNCount[ objID ] += 1;
	//				if(objNNCount[objID] > maxCount) {
	//					maxCount = objNNCount[objID];
	//					maxObj   = objID;
	//				}
	//			}
	//		}
	//		if(maxCount > 4) {
	//			CorrespondPoint c;
	//			c.dstID = i;
	//			c.srcID = indices_ptr[10*i];
	//			c.dist  = dists_ptr[10*i];
	//			c.objID = maxObj;
	//			correspondList.push_back(c);
	//			numAll++;
	//			numNN[c.objID]++;
	//			sumDist[c.objID] += dists_ptr[10*i];
	//		}
	//	}
	//}

	CorrespondPoint *correspond = new CorrespondPoint[numAll];
	int i = 0;
	for(vector<CorrespondPoint>::iterator it = correspondList.begin(); it != correspondList.end(); it++){
		correspond[i++] = *it;
	}


	for(int obj = 0; obj < indexBook->numObj; obj++) {
		//printf("%12s : %6d/%6d : %10.4f : %8.4f : %8.4f\n", indexBook->label[obj]
        //                                               , numNN[obj], indexBook->index->size()
		//											     , (float)indexBook->index->size()/(numNN[obj]+1)
		//												 , sumDist[obj]
		//												 , sumDist[obj]/(numNN[obj]+1));
		if(numNN[obj] > 5) {
			float sum_x = 0.0f;
			float sum_y = 0.0f;
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


				if(obj == 6){
					sum_x += surf->pt.x;
					sum_y += surf->pt.y;
					cvCircle(markImg, cvPoint(xx,yy), 1, colors[obj], 2);
					//printf("%s : (%3d, %3d, %3d) %.1f %.1f\n", colorImg->channelSeq, r_val, g_val, b_val,surf->pt.x,surf->pt.y);
				}else if(obj == 7){
					sum_x += surf->pt.x;
					sum_y += surf->pt.y;
					cvCircle(markImg, cvPoint(xx,yy), 1, colors[obj], 2);
					printf("%s : (%3d, %3d, %3d) %.1f %.1f\n", colorImg->channelSeq, r_val, g_val, b_val,surf->pt.x,surf->pt.y);
				}else{
					sum_x += surf->pt.x;
					sum_y += surf->pt.y;				
				}

				
				//printf("%.1f %.1f\n",surf->pt.x,surf->pt.y);
			}
			int avg_x = cvRound(sum_x/(numNN[obj]+1));
			int avg_y = cvRound(sum_y/(numNN[obj]+1));
			//int avg_x = cvRound(sum_x);
			//int avg_y = cvRound(sum_y);
			cvCircle(markImg, cvPoint(avg_x,avg_y), 1, colors[obj], 2);
			cvPutText(markImg, indexBook->label[obj], cvPoint(avg_x,avg_y), &cvFont(1.0, 2), colors[obj]);



		}
	}
	//printf("-----------------------------------------------\n");

	//for(int p = 0; p < numAll; p++) {
	//	CvSURFPoint *surf = (CvSURFPoint*)cvGetSeqElem(keypoints, correspond[p].dstID);
	//	cvCircle(markImg, cvPoint(surf->pt.x,surf->pt.y), 1, colors[correspond[p].objID], 2);
	//}

	delete correspond;
	delete queryVector;
}

void test(IndexBook *indexBook, char *queryFile)
{
	float nnRatio   = 0.3f;
	printf(" >> query from : %s\n", queryFile);
	IplImage *queryImg = cvLoadImage(queryFile, CV_LOAD_IMAGE_GRAYSCALE);
	IplImage *markImg  = cvCreateImage(cvSize(queryImg->width, queryImg->height), queryImg->depth, 3);
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

	//// mark surf
	//CvSeqReader seqReader;
	//cvStartReadSeq(keypoints, &seqReader );
	//for(int i = 0; i < desc->total; i++ ) {
	//	CvSURFPoint* surf = (CvSURFPoint*)seqReader.ptr;
	//	cvCircle(markImg, cvPoint(surf->pt.x,surf->pt.y), 1, colors[0], 2);
	//	CV_NEXT_SEQ_ELEM( seqReader.seq->elem_size, seqReader );
	//}


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
		printf("%12s : %6d/%6d : %10.4f : %8.4f : %8.4f\n", indexBook->label[obj]
                                                       , numNN[obj], indexBook->index->size()
													     , (float)indexBook->index->size()/(numNN[obj]+1)
														 , sumDist[obj]
														 , sumDist[obj]/(numNN[obj]+1));
		if(numNN[obj] >= 1) {
			float sum_x = 0.0f;
			float sum_y = 0.0f;
			for(int p = 0; p < numAll; p++) {
				if(correspond[p].objID != obj) {
					continue;
				}
				CvSURFPoint *surf = (CvSURFPoint*)cvGetSeqElem(keypoints, correspond[p].dstID);
				sum_x += surf->pt.x;
				sum_y += surf->pt.y;
				//cvCircle(markImg, cvPoint(surf->pt.x,surf->pt.y), 1, colors[obj], 2);
			}
			int avg_x = cvRound(sum_x/(numNN[obj]+1));
			int avg_y = cvRound(sum_y/(numNN[obj]+1));
			cvCircle(markImg, cvPoint(avg_x,avg_y), 1, colors[obj], 2);
			cvPutText(markImg, indexBook->label[obj], cvPoint(avg_x,avg_y), &cvFont(1.0, 2), colors[obj]);
		}
	}

	delete correspond;
	delete queryVector;
	cvShowImage("input", markImg);
	cvSaveImage("result.jpg", markImg);
	cvWaitKey();
}

int main()
{
	char *imgLibDir = "./img-lib";
	char *queryFile = "query.000.bmp";
	float nnRatio   = 0.3f;
	bool reindexing = true;
	bool camera_running = true;
	CvCapture* capture = 0;
	int frameWidth  = 0;
	int frameHeight = 0;
	
	char fileName[1024];
	int numSaveFrame = 0;
	int inKey = 0;
	int numObj = 0;

	if(reindexing || !is_updated(imgLibDir)) {
		printf("[Initialize] : reindexing\n");
		do_index(imgLibDir, KDTreeIndex);
	}

	if(camera_running) {
		cvNamedWindow("input", 1 );
		capture = cvCreateCameraCapture(CV_CAP_ANY+0);
		cvSetMouseCallback("input", on_mouse);
	}
	
	if(!capture && camera_running) {
		fprintf(stderr, "[ERROR][Initialize] : can not find device\n");
		exit(1);
	}
	frameWidth  = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	frameHeight = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);

	
	printf("[Initialize] : reading index\n");
	IndexBook *indexBook = load_index(imgLibDir);
	write_updated(imgLibDir, indexBook);

	if(!camera_running) {
		test(indexBook, "test-img/query.000.bmp");
		test(indexBook, "test-img/query.001.bmp");
		test(indexBook, "test-img/query.002.bmp");
		test(indexBook, "test-img/query.003.jpg");
		test(indexBook, "test-img/query.004.jpg");
		exit(0);
	}

	IplImage *grayImg  = cvCreateImage(cvSize(frameWidth, frameHeight), 8, 1);
	IplImage *markImg  = cvCreateImage(cvSize(frameWidth, frameHeight), 8, 3);
	int curObj = 0;
	bool editLib = false;
	while(camera_running)
	{
		IplImage *inFrame = cvQueryFrame(capture);
		
		cvCvtColor(inFrame, grayImg, CV_BGR2GRAY);

		//cvCvtColor(grayImg, markImg, CV_GRAY2RGB);

		findObjectAndMark(grayImg, markImg, indexBook, inFrame);

		//output
		cvDrawRect(markImg, cvPoint(gROI_x1,gROI_y1), cvPoint(gROI_x2,gROI_y2), CV_RGB(0,255,0));
		cvPutText(markImg, indexBook->label[curObj], cvPoint(gROI_x1,gROI_y1), &cvFont(1.5,2), CV_RGB(255,255,255));
		cvShowImage("input", markImg);

		inKey = cvWaitKey(1);
		if(inKey == 27){
			break;
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
		else if(inKey >= 0){
			printf("key = %d\n", inKey);
		}
	}

	if(editLib) {
		write_edited(imgLibDir, indexBook);
	}

	cvReleaseImage(&grayImg);
	cvReleaseImage(&markImg);
	cvDestroyAllWindows();
	cvReleaseCapture(&capture);
	return 0;
}