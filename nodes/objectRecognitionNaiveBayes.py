#!/usr/bin/python
from sklearn import neighbors, datasets
import roslib
roslib.load_manifest('object_perception')
import rospy

from std_msgs.msg import String
from object_perception.srv import *
from operator import add

class objectRecognition:

	def __init__(self):
		rospy.init_node('ObjectRecognitionNaivebayes', anonymous=True)
		rospy.Subscriber("featureFilePath",String,self.classifyObjectHandle)
		self.recognitionResult = rospy.Publisher('recognitionResult',String) 

		learningListFile = "/home/skuba/skuba_athome/object_perception/learn/LocalizationTrain/15.train"
		feature,self.labels = self.loadTrainData(learningListFile)
		K_neighbors = int(rospy.get_param('~k_neighbors', 35))
		self.clf = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
		self.clf.fit(feature, self.labels) 

		rospy.Service('classifyObject',classifyObject,self.classifyObjectService)

		rospy.loginfo('Verification Start')
		rospy.spin()
		
	def loadTrainData(self,filename):
		self.categorySet = {}
		self.revertCategory = []
		feature_list = []
		label_list = []
	   	filePtr = open(filename,"r")
		for line in filePtr:
			category,filePath = line.strip().split(",")
			if not category in self.categorySet:
				self.categorySet[category] = len(self.categorySet)
				self.revertCategory.append(category)
			feature,label = self.loadFeature(filePath,self.categorySet[category])
			feature_list += feature
			label_list += label
	    	return feature_list,label_list

	def loadFeature(self,filename,category):
		feature_list = []
		label_list = []
		filePtr = open(filename,"r")
		for line in filePtr:
			attribute = line.strip().split(",")
			feature_list.append(map(float,attribute))
			label_list.append(int(category))
		filePtr.close()
		return feature_list,label_list

	def classifyObjectHandle(self,data):
		category = self.predictObject(data.data)
		self.recognitionResult.publish(String(self.revertCategory[category]))

	def classifyObjectService(self,req):
		category = self.predictObject(req.filepath)
		return classifyObjectResponse(category)

	def predictObject(self,featureFileName):
		queryFeature,label = self.loadFeature(featureFileName,'-1')
		if len(queryFeature) == 0:
			return -1

		weightSum = [0.0 for i in self.categorySet]
		for aFeature in queryFeature:
			weight = self.predictFeature(aFeature)
			weightSum = map(add, weight, weightSum)

		return int(weightSum.index(min(weightSum)))

	def predictFeature(self,feature):
		classSet = []
		weight = [0.0 for i in self.categorySet]
		dis,ind = self.clf.kneighbors(feature)
		dis = dis[0]
		ind = ind[0]
		for x in range(len(ind)):
			if(self.labels[ind[x]] in classSet):
				continue
			classSet.append(self.labels[ind[x]])
			weight[self.labels[ind[x]]] = dis[x] - dis[-1]
		return weight
 
if __name__ == '__main__':
    try:
	rospy.loginfo('Verification Node Start Initializing')
        objectRecognition()
    except rospy.ROSInterruptException:
	rospy.logerror('Error ROS')
