#!/usr/bin/python
from sklearn import neighbors, datasets
import roslib
import os
roslib.load_manifest('object_recognition')

import rospy
import cv2

from std_msgs.msg import String
from object_recognition.srv import *
from operator import add


surf = cv2.SURF(400)

class object_recognition:

    def __init__(self):
        rospy.init_node('verify_object_service')
        rospy.Subscriber("/object_recognition/create_model", String, self.create_model)
        
        feature_directory = rospy.get_param('~feature_directory', '/config/feature')
        K_neighbors = int(rospy.get_param('~k_neighbors', 35))

        feature, self.labels = self.load_train_data(feature_directory)
        self.clf = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
        self.clf.fit(feature, self.labels) 

        object_threshold_file = rospy.get_param('~threshold_file', 'config/threshold.txt')
        object_threshold_file = "/home/skuba/skuba_athome/object_perception/object_recognition/config/result_threshold.txt"
        self.object_threshold = self.read_object_threshold(object_threshold_file)
        #print self.object_threshold
        
        rospy.Service('/object_recognition/verify_object', classifyObject, self.classify_object_service)
        
        rospy.loginfo('Verify node Start')
        rospy.spin()
		
    def read_object_threshold(self,file_name):
        file_ptr = open(file_name,"r")
        threshold_list = {}
        for line in file_ptr:
            object_name, threshold = line.split()
            threshold_list[object_name] = float(threshold)
        threshold_list['unknown'] = 1.0
        return threshold_list

    def load_train_data(self, feature_directory):
        feature_list = []
        label_list = []
        self.category_set = []
        for feature_file in os.listdir(feature_directory):
            if feature_file.endswith(".txt"):
                category = os.path.splitext(os.path.basename(feature_file))[0]
                self.category_set.append(category)
                category_index = self.category_set.index(category)
                feature, label = self.load_feature(os.path.join(feature_directory,feature_file), category_index)
                
                feature_list += feature
                label_list += label
                
        self.category_set.append('unknown')
        return feature_list,label_list

    def load_feature(self, filename, category):
        feature_list = []
        label_list = []
        file_feature = open(filename,"r")
        for line in file_feature:
            attribute = line.strip().split()
            feature_list.append(map(float,attribute))
            label_list.append(int(category))
        file_feature.close()
        return feature_list,label_list
    
    def create_model(self,data):
        print 'in create_model'
        feature,self.labels = self.loadTrainData(data.data)
        self.clf = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
        self.clf.fit(feature, self.labels) 

    def classify_object_service(self, req):
        print "incoming input :", req.filepath
        category, confident = self.predict_object(req.filepath)
        #return classifyObjectResponse(str(self.category_set[category]),confident)
        if self.object_threshold[str(self.category_set[category])] > confident:
            return classifyObjectResponse(str(self.category_set[category]),confident)
        print 'predicted result(wrong) as',str(self.category_set[category]),'with confident',confident
        return classifyObjectResponse('unknown',confident)
    
    def predict_object(self, image_filename):
        image = cv2.imread(image_filename, 0)
        keypoint, features = surf.detectAndCompute(image, None)

        if features == None or len(keypoint) == 0:
            return -1, 0.0

        query_feature = [map(float,feature) for feature in features]

        weight_sum = [0.0 for i in self.category_set]
        for feature in query_feature:
            weight = self.predict_feature(feature)
            weight_sum = map(add, weight, weight_sum)
            
        sortedWeightSum = sorted(weight_sum)
        print "filename :", image_filename , "with difference :",str(sortedWeightSum[0]-sortedWeightSum[1])
        return int(weight_sum.index(min(weight_sum))), (sortedWeightSum[0]-sortedWeightSum[1])
    
    def predict_feature(self, feature):
        class_set = []
        weight = [0.0 for i in self.category_set]
        dis,ind = self.clf.kneighbors(feature)
        dis = dis[0]
        ind = ind[0]
        for x in range(len(ind)):
            if(self.labels[ind[x]] in class_set):
                continue
            class_set.append(self.labels[ind[x]])
            weight[self.labels[ind[x]]] = dis[x] - dis[-1]
        return weight
 
if __name__ == '__main__':
    try:
        rospy.loginfo('Verification Node Start Initializing')
        object_recognition()
    except rospy.ROSInterruptException:
        rospy.logerror('Error ROS')

