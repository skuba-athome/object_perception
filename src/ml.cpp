#include<iostream>
#include<math.h>
#include<string>
#include<algorithm>
#include"cv.h"
#include"ml.h"
#include"highgui.h"

using namespace std;
using namespace cv;

int training_point = 5147;
int testing_point = 532;
int numObject = 7;
string train_file = "/home/skuba/Downloads/libsvm-3.16/tools/train_data";
string test_file = "/home/skuba/Downloads/libsvm-3.16/tools/train_data.t";

static int layer_perm = 0;

cv::Mat layers_gen(int i) {
	cv::Mat layers;
	if(i < 3){ layers = cv::Mat(3,1,CV_32SC1);layers.row(2) = cv::Scalar(numObject);}
	else if(i < 12){ layers = cv::Mat(4,1,CV_32SC1);layers.row(3) = cv::Scalar(numObject);}
	else{ layers = cv::Mat(5,1,CV_32SC1);layers.row(4) = cv::Scalar(numObject);}
	layers.row(0) = cv::Scalar(128);
	switch(i%3) {
		case 0: layers.row(1) = cv::Scalar(128);break;
		case 1: layers.row(1) = cv::Scalar(256);break;
		case 2: layers.row(1) = cv::Scalar(512);break;
	}
	if(i > 2){
		i = i/3;
		switch(i%3) {
			case 0: layers.row(2) = cv::Scalar(512);break;
			case 1: layers.row(2) = cv::Scalar(128);break;
			case 2: layers.row(2) = cv::Scalar(256);break;
		}
	}
	if(i > 2){
		i = i/3;
		switch(i%3) {
			case 0: layers.row(3) = cv::Scalar(512);break;
			case 1: layers.row(3) = cv::Scalar(128);break;
			case 2: layers.row(3) = cv::Scalar(256);break;
		}
	}
	return layers;
}


//classify function
int classify(cv::Mat result) {
	float max = 0.0f;
	int max_i = 0;
	for(register int i=0;i<numObject;++i) {
		if(result.at<float>(0,i) > max) {
			max = result.at<float>(0,i);
			max_i = i;
		}
	}
	return max_i;
}

//evaluate function
float evaluate(cv::Mat& predicted,cv::Mat& actual) {
	assert(predicted.rows == actual.rows);
	int t=0,f=0;
	for(register int i=0;i<actual.rows;++i) {
		int predictClass = classify(predicted.row(i));
		int actualClass = classify(actual.row(i));
		if(predictClass == actualClass) ++t;
		else ++f;
	}
	//cout << predicted << endl;
	//cout << actual << endl;
	return (t*1.0)/(t+f);
}

///using neuron-network

void mlp(cv::Mat& trainingData,cv::Mat& trainingClass,cv::Mat& testData,cv::Mat& testClass) {
	cv::Mat layers = layers_gen(layer_perm++);
	//cout << layers << endl;
	//return;
	
	CvANN_MLP mlp;
	CvANN_MLP_TrainParams params;
	CvTermCriteria criteria;
	criteria.max_iter = 100000;
	criteria.epsilon = 0.00001f;
	criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
	params.train_method = CvANN_MLP_TrainParams::BACKPROP;
	params.bp_dw_scale = 0.05f;
	params.bp_moment_scale = 0.05f;
	params.term_crit = criteria;

	mlp.create(layers);
	
	mlp.train(trainingData,trainingClass,cv::Mat(),cv::Mat(),params);
	
	cv::Mat predicted(testClass.rows,numObject,CV_32F);
	for(register int i=0;i<testClass.rows;++i) {
		cv::Mat response(1,numObject,CV_32FC1);
		cv::Mat sample = testData.row(i);
		
		mlp.predict(sample,response);
		for(register int j=0;j<numObject;++j)
			predicted.at<float>(i,j) = response.at<float>(0,j);
	}
	
	cout << layers << endl << "Accuracy :" << evaluate(predicted,testClass) << endl;
}

void read_data(cv::Mat& feature,cv::Mat& classes,string filename,int numPoint) {
	FILE *f = fopen(filename.data(),"r");
	if(f == NULL){cout << "Error no file" << endl;return;}
	for(register int i=0;i<numPoint;i++) {
		register int _class;
		fscanf(f,"%d",&_class);
		classes.at<float>(i,_class) = 1.0f;
		for(register int j=0;j<128;j++) {
			float temp;
			fscanf(f,"%d:%f",&_class,&temp);
			feature.at<float>(i,_class-1) = temp;
		}
	}
	fclose(f);
}

int main() {
	//data
	cv::Mat trainingData(training_point,128,CV_32FC1);
	cv::Mat testData(testing_point,128,CV_32FC1);
	//class
	cv::Mat trainingClass(training_point,numObject,CV_32FC1);
	cv::Mat testClass(testing_point,numObject,CV_32FC1);
	
	read_data(trainingData,trainingClass,train_file,training_point);
	read_data(testData,testClass,test_file,testing_point);
	
	for(int i=0;i<30;i++)
	mlp(trainingData,trainingClass,testData,testClass);
	
	return 0;
}

