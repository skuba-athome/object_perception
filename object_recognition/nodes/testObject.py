#!/usr/bin/python

from sklearn import neighbors, datasets
from sklearn.externals import joblib
import cv2
import os
import numpy
import roslib
import rospy

from operator import add
from std_msgs.msg import String

object_root_dir = roslib.packages.get_pkg_dir('object_recognition') + '/data/'
object_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/object_names.txt'
features_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/Features/'
nb_model_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/Nb_model/nb_model'
#object_image_dir = roslib.packages.get_pkg_dir('tabletop') + '/out'
object_image_dir = roslib.packages.get_pkg_dir('object_recognition') + '/data/pringles/train'


surf = cv2.SURF(400)

class objectRecognition:

    def __init__(self):
        self.object_names = [line.rstrip('\n') for line in open(object_filename)]
        features, self.labels  = self.loadObjectFeatures(self.object_names)        
        
        #print self.labels

        image_dic = [ str(os.path.join(object_image_dir, image)) for image in os.listdir(object_image_dir) if image.endswith(".png")]
        self.nb_model = joblib.load(nb_model_filename)        
        K_neighbors = 35
        self.nb_model = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
        self.nb_model.fit(features, self.labels) 
        
        
        for aImage in image_dic:
            nb_answer, confidence = self.predictObject(aImage)
        #nb_answer, confidence = self.predictObject(image_dic[0])            
    
    def loadObjectFeatures(self, object_names):
        features = []
        labels = []
        cnt = 0
        for name in object_names:
            print name, cnt
            feature_file = str(features_filename) + str(name) + ".txt"
            filePtr = open(feature_file,"r")
            feature_list = []
            label_list = []
            for line in filePtr:
                attribute = line.strip().split(" ")
                feature_list.append(map(float,attribute))
                label_list.append(cnt)
            filePtr.close()
            features += feature_list
            labels += label_list
            cnt+=1
            
        return features, labels
    
    def predictObject(self, image_filename):
        image = cv2.imread(image_filename, 0)
        print image_filename
        keypoint, features = surf.detectAndCompute(image, None)        
        if features == None or len(features) == 0:
            return -1
        queryFeature = [map(float,feature) for feature in features]
        weightSum = [0.0 for i in self.object_names]

        for aFeature in queryFeature:
            weight = self.calculateFeatureWeight(aFeature)
            weightSum = map(add, weight, weightSum)
        #print weightSum
        nb_answer = int(weightSum.index(min(weightSum)))
        sortedWeightSum = sorted(weightSum)
        confidence = sortedWeightSum[0]-sortedWeightSum[1]
        print nb_answer, confidence
        return nb_answer, confidence

    def calculateFeatureWeight(self,feature):
        weight = [0.0 for i in self.object_names]
        distance, index = self.nb_model.kneighbors(feature)
        distance = distance[0]
        index = index[0]
        for x in range(len(index)): 
            weight[self.labels[index[x]]] = distance[x] - distance[-1]
            #print self.labels[index[x]]
        #print weight
        return weight    

if __name__ == '__main__':
    rospy.init_node('testObject')
    objectRecognition()