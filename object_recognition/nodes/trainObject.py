#!/usr/bin/python

import roslib
import rospy

from sklearn import neighbors, datasets
from sklearn.externals import joblib
from sklearn.svm import SVC
import cv2
import os
import numpy
from operator import add

surf = cv2.SURF(400)

object_root_dir = roslib.packages.get_pkg_dir('object_recognition') + '/data/'
object_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/object_names.txt'
features_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/Features/'
#nb_model_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/Nb_model/nb_model'
#svm_model_filename = roslib.packages.get_pkg_dir('object_recognition') + '/config/svm_model.pkl'

class objectRecognition:

	def __init__(self):
		
		object_dic = self.listImageInDirectory(object_root_dir)
		features_list, self.labels = self.extractFeatures(object_dic)
		
		K_neighbors = 35
		self.clf = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
		self.clf.fit(features_list, self.labels)
		#joblib.dump(self.clf, nb_model_filename) 
		
		#self.trainSVM(object_dic)
		#self.loadFeature(features_filename, category)
		#testing
		#for queryFeature in features_list:
		#	weightSum = self.calWeightSum(queryFeature)
		#	self.svm(histogram, category, weightSum)


	def listImageInDirectory(self, dir):
		image_dic = {}
		#filename = open(object_filename, 'w')
		for object_dir in os.listdir(dir):
		    #filename.write(str(object_dir) + '\n')
		    object_dir_path = os.path.join(dir, object_dir + '/test/')
		    if not os.path.isdir(object_dir_path):
		        continue
		    image_dic[str(object_dir)] = []
		    for object_pic in os.listdir(object_dir_path):
		        if object_pic.endswith(".jpg") or object_pic.endswith(".png"):
		            image_dic[str(object_dir)].append(str(os.path.join(object_dir_path, object_pic)))
		#filename.close()
		return image_dic


	def extractFeatures(self, object_dic):
		self.aFeature = []
		self.aLabel = []
		self.categorySet = []
		filename = open(object_filename, 'w')
		for object_name in object_dic:
			self.features = []			
			category = object_name
			if not category in self.categorySet:
				self.categorySet.append(category)
				filename.write(str(category) + '\n')
			for aImage in object_dic[object_name]:
				feature = self.extractFeatureFromAImage(aImage, object_name)
				if feature == None or len(feature) == 0:
					continue            
				for f in feature:
					self.aFeature.append(map(float,f))
					self.features.append(map(float,f))
					self.aLabel.append(len(self.categorySet)-1)			
			#self.writeFeatureFile(os.path.join(object_root_dir, object_name +"/features.txt"), self.features)
			self.writeFeatureFile(os.path.join(features_filename + object_name + ".txt"), self.features)
		filename.close()
		return self.aFeature, self.aLabel

	def extractFeatureFromAImage(self, aImage, object_name):
		image = cv2.imread(aImage, 0)
		keypoint, feature = surf.detectAndCompute(image, None)
		basename = os.path.splitext(os.path.basename(aImage))[0]
		#self.writeFeatureFile(os.path.join(object_root_dir, object_name +"/feature/"+ basename +".txt"), feature)
		return feature

	def writeFeatureFile(self, file_name, feature):
		file_ptr = open(file_name, 'w')
		for aFeature in feature:
			line = map((lambda x: '%.15f' % (x)), aFeature)
			line = reduce((lambda x, y: '%s %s' % (x,y)), line)
			file_ptr.write(line+'\n')
		file_ptr.close()
		print 'Write Feature File: '+file_ptr.name+' completed'
	
	def calWeightSum(self, queryFeature):
		weightSum = [0.0 for i in self.categorySet]
		if queryFeature == None or len(queryFeature) == 0:
			return 1000000
		for aFeature in queryFeature:
			weight = self.calWeight(aFeature)
			weightSum = map(add, weight, weightSum)
		return weightSum

	def calWeight(self,feature):
		classSet = []
		weight = [0.0 for i in self.categorySet]

		dis, ind = self.clf.kneighbors(feature)
		dis = dis[0]
		ind = ind[0]

		for x in range(len(ind)):
			if(self.labels[ind[x]] in classSet):
				continue        
			classSet.append(self.labels[ind[x]])            
			weight[self.labels[ind[x]]] = dis[x] - dis[-1]
		return weight
	
	def trainSVM(self, object_dic):
		histogram = []
		category = []
		index = 0
		for object_name in object_dic:
			for aImage in object_dic[object_name]:
				queryFeature = self.extractFeatureFromAImage(aImage, object_name)
				weightSum = self.calWeightSum(queryFeature)	
				histogram.append(weightSum)
				#print weightSum, index
				category.append(index)
			print object_name, index
			index += 1
		X = numpy.array(histogram)
		y = numpy.array(category)

		#clf_svm = SVC()
		clf_svm = SVC(kernel='linear')
		#clf_svm = SVC(kernel='rbf',C=3.0)
		#clf_svm = SVC(kernel='rbf',C=5.0)
		#clf_svm = SVC(kernel='polynomial')
		#clf_svm = SVC(kernel='sigmoid')

		clf_svm.fit(X, y) 
		#print clf_svm.predict([-2.423903539202386, -1.2952946407463655, -13.331718175570442])		
		joblib.dump(clf_svm, svm_model_filename) 

if __name__ == '__main__':
	rospy.init_node('trainObject')
	objectRecognition()