#!/usr/bin/python
from sklearn import neighbors, datasets
import roslib
roslib.load_manifest('object_recognition')
import rospy
import cv2

from std_msgs.msg import String
from object_recognition.srv import *
from operator import add


surf = cv2.SURF(400)

class objectRecognition:

    def __init__(self):
        rospy.init_node('ObjectRecognitionNaivebayes', anonymous=True)
        rospy.Subscriber("featureFilePath",String,self.classifyObjectHandle)
        self.recognitionResult = rospy.Publisher('recognitionResult',String) 
        
        learningListFile = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/15.train"
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
        print filename
        filePtr = open(filename,"r")
        for line in filePtr:
            print line
            #category,filePath = line.strip().split(",")
            category,filePath = line.strip().split(" ")
            print filePath
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
        #print data,'--------------------'
        category = self.predictObject(data.data)
        print String(self.revertCategory[category])
        self.recognitionResult.publish(String(self.revertCategory[category]))
    
    def classifyObjectService(self,req):
        print "incoming input :",req.filepath
        category,confident = self.predictObject(req.filepath)
        return classifyObjectResponse(str(self.revertCategory[category]),confident)
    
    def predictObject(self,featureFileName):
        #queryFeature,label = self.loadFeature(featureFileName,'-1')
        #print 'type(queryFeature) = ',type(queryFeature[0]), 'type(label) = ',type(label[0])

        image = cv2.imread(featureFileName, 0)
        #queryFeature,label = surf.detectAndCompute(image, None)
        queryFeature = []
        #label = []
        keypoint, features = surf.detectAndCompute(image, None)

        print len(keypoint)

        if features == None or len(features) == 0:
            return -1

        for feature in features:
            queryFeature.append(map(float,feature))

        weightSum = [0.0 for i in self.categorySet]
        for aFeature in queryFeature:
            weight = self.predictFeature(aFeature)
            weightSum = map(add, weight, weightSum)
            
        #print min(weightSum)

        sortedWeightSum = sorted(weightSum)
        print "filename :",featureFileName , "with difference :",str(sortedWeightSum[0]-sortedWeightSum[1])

        return int(weightSum.index(min(weightSum))), (sortedWeightSum[0]-sortedWeightSum[1])
    
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

