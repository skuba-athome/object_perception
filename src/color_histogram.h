#ifndef _COLOR_HISTOGRAM_H_
#define _COLOR_HISTOGRAM_H_

#include <iostream>
#include <string>
#include <set>
#include <algorithm>

using namespace std;

typedef struct color_hist {
	// 20 : white , 21 : black
	static const float saturate_threshold = 0.2f; // white
	static const float value_threshold = 50.0f; // black
	int hist[22];
	float hist_norm[22];
	void init () {
		for(register int i=0;i<22;++i) hist[i]=0;
	}

	inline int h_classify(float h){
		for(register int i=0;i<19;++i)
			if(h < i*18+6.0) return i;
		return 0;
	}

    void classify(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud) {
		 for(size_t i=0;i<cloud->points.size();++i) {
		 	 classify(cloud->points[i].h,cloud->points[i].s,cloud->points[i].v);
	 	 }
	}
	
	void classify(float h,float s,float v) {
		if(s < saturate_threshold) hist[20]++;
		else if(v < value_threshold) hist[21]++;
		else hist[h_classify(h)]++;
	}

	void norm() {
		//int hist_max = 0;
		//for(register int i=0;i<22;++i) hist_max = max(hist_max,hist[i]);
		for(register int i=0;i<22;++i) hist_norm[i] = hist[i]*1.0/count();
	}
	
	inline float operator-(struct color_hist &b) {
		norm(); b.norm();
		float sum = 0.0f;
		for(register int i=0;i<22;++i) sum += (hist_norm[i] - b.hist_norm[i])*(hist_norm[i] - b.hist_norm[i]);
		return sqrt(sum);
	}

	friend ostream& operator<<(ostream& os,const color_hist &a) {
		for(register int i=0;i<22;++i) os << "hist[" << i << "] : " << a.hist[i] << endl;
		return os;
	}
	
	void PrintNorm() {
		norm();
		for(register int i=0;i<22;++i) cout << "hist_norm[" << i << "] : " << hist_norm[i] << endl;
	}

	int count() {
		int sum = 0;
		for(register int i=0;i<22;++i) sum+= hist[i];
		return sum;
	}
	
	void writeFile(string filename) {
		FILE *f_ptr = fopen(filename.data(),"w");
		fprintf(f_ptr,"%d\n",count());
		for(register int i=0;i<22;++i) fprintf(f_ptr,"%d\n",hist[i]);
	}
	
	void readFile(string filename) {
		FILE *f_ptr = fopen(filename.data(),"r");
		if(f_ptr == NULL) return;
		fscanf(f_ptr,"%d",&hist[0]);
		for(register int i=0;i<22;++i) fscanf(f_ptr,"%d",&hist[i]);
	}

} color_hist;
#endif
