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

static int layer_perm = 2;

cv::Mat layers_gen(int i = -1) {
	cv::Mat layers;
	if(i > 0){
		layers = cv::Mat(3,1,CV_32SC1);
		
		layers.row(0) = cv::Scalar(128);
		//layers.row(1) = cv::Scalar(256);
		layers.row(1) = cv::Scalar(i);
		layers.row(2) = cv::Scalar(numObject);
		return layers;
	}
	/*if(i < 3){ layers = cv::Mat(3,1,CV_32SC1);layers.row(2) = cv::Scalar(numObject);}
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
	}*/
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
		int predictClass,actualClass;
		if(predicted.cols == 1) {
			predictClass = predicted.at<float>(i,0);
			actualClass = actual.at<float>(i,0);
		} else {
			predictClass = classify(predicted.row(i));
			actualClass = classify(actual.row(i));
		}
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
	//cv::Mat layers = layers_gen();
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
	params.bp_moment_scale = 0.1f;
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

// KNN

void knn(cv::Mat& trainingData,cv::Mat& trainingClass,cv::Mat& testData,cv::Mat& testClass,int K) {
	CvKNearest knn(trainingData,trainingClass,cv::Mat(),false,K);
	cv::Mat predicted(testClass.rows,1,CV_32F);
	for(int i=0;i < testData.rows;++i) {
		const cv::Mat sample = testData.row(i);
		predicted.at<float>(i,0) = knn.find_nearest(sample,K);
	}
	//cout << predicted << endl;
	cout << "Accuracy :" << evaluate(predicted,testClass) << endl;
	
}


void read_data(cv::Mat& feature,cv::Mat& classes,string filename,int numPoint) {
	FILE *f = fopen(filename.data(),"r");
	if(f == NULL){cout << "Error no file" << endl;return;}
	for(register int i=0;i<numPoint;i++) {
		int _class;
		fscanf(f,"%d",&_class);
		classes.at<float>(i,_class-1) = 1.0f;
		for(register int j=0;j<128;j++) {
			float temp;
			fscanf(f,"%d:%f",&_class,&temp);
			feature.at<float>(i,_class-1) = temp;
		}
	}
	fclose(f);
}

float findDistance(cv::Mat first,cv::Mat second) {
	float sum = 0.0;
	for(register int i=0;i<first.cols;++i) {
		sum += (first.at<float>(0,i)-second.at<float>(0,i))*(first.at<float>(0,i)-second.at<float>(0,i));
	}
	return sqrt(sum);
}

int countMatches(cv::Mat screen,cv::Mat object) {
	float dist,d1,d2;
	int countMatch = 0;
	for(int i=0;i<object.rows;++i) {
		d1 = d2 = FLT_MAX;
		for(int j=0;j<screen.rows;++j) {
			dist = findDistance(object.row(i),screen.row(j));
			
			if(dist < d1) {
				d2 = d1;
				d1 = dist;
			} else if(dist < d2) {
				d2 = dist;
			}
		}
		if(d1/d2 < 0.65) countMatch++;
	}
	return countMatch;
}

cv::Mat getObjectPoint(cv::Mat data,cv::Mat _class,int object) {
	int count = 0;
	for(int i=0;i<_class.rows;++i) {
		if(_class.at<float>(i,object) > 0.5) ++count;
	}
	//cout << count << endl;
	cv::Mat result(count,128,CV_32FC1);
	int temp = 0;
	for(int i=0;i<_class.rows;++i) {
		if(_class.at<float>(i,object) > 0.5) {
			//cout << i <<" "  << " " << temp <<endl;
			for(int j=0;j<128;++j) {
				result.at<float>(temp,j) = data.at<float>(i,j);
			}
			++temp;
		}
	}
	return result;
}

cv::Mat to1Dimension(cv::Mat data) {
	cv::Mat result(data.rows,1,CV_32FC1);
	for(int i=0;i<data.rows;++i) {
		int max = 0;
		for(int j=1;j<data.cols;++j){
			if(data.at<float>(i,j) > data.at<float>(i,max))
				max = j;
		}
		result.at<float>(i,0) = max;
	}
	return result;
}

int main(int argc,char **argv) {
	//data
	cv::Mat trainingData(training_point,128,CV_32FC1);
	cv::Mat testData(testing_point,128,CV_32FC1);
	//class
	cv::Mat trainingClass(training_point,numObject,CV_32FC1);
	cv::Mat testClass(testing_point,numObject,CV_32FC1);
	
	read_data(trainingData,trainingClass,train_file,training_point);
	read_data(testData,testClass,test_file,testing_point);
	
	/*for(int i=0;i<7;++i) {
		cv::Mat temp1 = getObjectPoint(trainingData,trainingClass,i);
		cv::Mat temp2 = getObjectPoint(testData,testClass,i);
		cout << i+1 << ":"  << countMatches(temp1,temp2) << endl;
	}*/
	for(int i=0;i<512;i++)
	mlp(trainingData,trainingClass,trainingData,trainingClass);
	/*trainingClass = to1Dimension(trainingClass); testClass = to1Dimension(testClass);
	for(int i=1;i<33;++i){
		cout << i << " " ;
		knn(testData,testClass,trainingData,trainingClass,i);
		//knn(trainingData,trainingClass,trainingData,trainingClass,i);
	}*/
	
	return 0;
}

