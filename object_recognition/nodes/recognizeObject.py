#!/usr/bin/python

from sklearn import neighbors, datasets
from sklearn.externals import joblib
import roslib; 
roslib.load_manifest('tabletop')
roslib.load_manifest('object_recognition')
from operator import add
from std_msgs.msg import String
#from tabletop.msg import Table
from tabletop.srv import TabletopObjectDetection
from object_recognition.srv import Recognize, RecognizeResponse
import cv2
import os
import numpy as np
import roslib
import rospy
from array import array
import shutil

#K_neighbors = 35
#object_root_dir = roslib.packages.get_pkg_dir('object_recognition') + '/data/'
#features_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/Features/'
#object_image_dir_in = roslib.packages.get_pkg_dir('object_recognition')

object_filename = roslib.packages.get_pkg_dir('object_recognition') + '/learn/object_names.txt'
#features_filename = "/home/mukda/Desktop/result/Features_full/"
#object_root_dir = "/home/mukda/Desktop/full"
#object_image_dir_in = "/home/mukda/Desktop/full_test"
#object_image_dir_out = "/home/mukda/Desktop/result/out_full"

surf = cv2.SURF(400)

class objectRecognition:

    def __init__(self):
        rospy.init_node('objectRecognition', anonymous=True)
        rospy.wait_for_service('tabletop_object_detection')        
        try:
            self.callback = rospy.ServiceProxy('tabletop_object_detection', TabletopObjectDetection)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        K_neighbors = int(rospy.get_param('~k_neighbors', 35))
        self.features_filename = rospy.get_param('~features_filename', roslib.packages.get_pkg_dir('object_recognition') + '/learn/Features/')
        self.object_image_dir_in = rospy.get_param('~object_image_dir_in', roslib.packages.get_pkg_dir('tabletop') + '/out')
        self.object_image_dir_out = rospy.get_param('~object_image_dir_out', roslib.packages.get_pkg_dir('object_recognition') + '/out')
        
        #object_dic = self.listImageInDirectory(object_root_dir)#, '/test/')
        object_dic =  self.listImageFromFilename()
        #print object_dic
        feature, self.labels = self.loadTrainData(object_dic)
        self.clf = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
        self.clf.fit(feature, self.labels) 

        '''
        image_dic = [ str(os.path.join(object_image_dir_in, image)) for image in os.listdir(object_image_dir_in) if image.endswith(".png") or image.endswith(".jpg")]
        image_dic.sort()
        for aImage in image_dic:
            print aImage
            cate, diff = self.predictObject(aImage)
            print "\t",cate, diff
        '''
        
        rospy.Service('object_recognition', Recognize, self.recognizeObjectService)
        rospy.loginfo('Object Recognition Node Ready')
        rospy.spin()        
        

    def recognizeObjectService(self, req):    
        response = self.callback()
        result = response.result
        centriods = response.centriods
        solid_boxes = response.solid_boxes
        clusters = response.clusters
        table = response.table
        
        image_dic = [ str(os.path.join(self.object_image_dir_in, image)) for image in os.listdir(self.object_image_dir_in) if image.endswith(".png")]
        image_dic.sort()

    	if not os.path.exists(self.object_image_dir_out):
    		os.makedirs(self.object_image_dir_out)
    	else:
    		shutil.rmtree(self.object_image_dir_out)
		
        names = []
        confidences = []
        cnt = 0
        for aImage in image_dic:
            print aImage
            cate, diff = self.predictObject(aImage)
            names.append(cate)
            confidences.append(diff)
            
            object_image_name = self.object_image_dir_out + "/object_" + str(names[cnt]) + ".png"
            with open(aImage, 'rb') as f:
                data = f.read()
            with open(object_image_name, 'wb') as f:
                f.write(data)
            cnt+=1
        print result
        print names
        print confidences

        if not os.path.exists(self.object_image_dir_in):
            os.makedirs(self.object_image_dir_in)
        else:
            shutil.rmtree(self.object_image_dir_in)

        return RecognizeResponse(result, names, confidences, clusters)
        #, centriods, solid_boxes, clusters)#, table)


    def listImageInDirectory(self):
        image_dic = {}
        for object_dir in os.listdir(dir):
            object_dir_path = os.path.join(dir, object_dir)# + name
            if not os.path.isdir(object_dir_path):
                continue
            image_dic[str(object_dir)] = []
            for object_pic in os.listdir(object_dir_path):
                if object_pic.endswith(".jpg") or object_pic.endswith(".png"):
                    image_dic[str(object_dir)].append(str(os.path.join(object_dir_path, object_pic)))
        return image_dic

    def listImageFromFilename(self):
        filePtr = open(object_filename,"r")
        object_names = [line.strip() for line in filePtr]
        filePtr.close()
        #print object_names
        return object_names

    def loadTrainData(self, object_dic):
        self.categorySet = {}       #dict
        self.revertCategory = []    #array
        feature_list = []
        label_list = []
        for object_name in object_dic:
            file_name = self.features_filename + object_name + ".txt"
            category = object_name
            if not category in self.categorySet:
                self.categorySet[category] = len(self.categorySet)
                self.revertCategory.append(category)
            feature,label = self.loadFeature(file_name, self.categorySet[category])
            feature_list += feature
            label_list += label
        return feature_list,label_list


    def loadFeature(self,filename,category):
        feature_list = []
        label_list = []
        filePtr = open(filename,"r")
        for line in filePtr:
            attribute = line.strip().split(" ")            
            feature_list.append(map(float,attribute))
            label_list.append(int(category))
        filePtr.close()
        return feature_list,label_list

    # this is Main function
    def predictObject(self,featureFileName):
        image = cv2.imread(featureFileName, 0)
        queryFeature = []
        keypoint, features = surf.detectAndCompute(image, None)        
        weightSum = [0.0 for i in self.categorySet]
        if features == None or len(features) == 0:
            return "Not found object", -5.00
        for feature in features:
            queryFeature.append(map(float,feature))
        for aFeature in queryFeature:
            weight = self.predictFeature(aFeature)
            weightSum = map(add, weight, weightSum)
        
        #print weightSum
        #print self.revertCategory

        sortedWeightSum = sorted(weightSum)
        nb_answer_num = int(weightSum.index(min(weightSum)))
        confident = sortedWeightSum[0]-sortedWeightSum[1]

        #print "\tpercent", confident/sortedWeightSum[0]
        i = 0        
        while (confident/sortedWeightSum[i]) <0.1:
            confident = sortedWeightSum[i]-sortedWeightSum[i+1]
            nb_answer_num = int(weightSum.index( sortedWeightSum[i] ))
            i+=1
            print confident, str(self.revertCategory[nb_answer_num])

        nb_answer = str(self.revertCategory[nb_answer_num])
        
        return nb_answer, confident

    def predictFeature(self,feature):
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

if __name__ == '__main__':
    try:        
        rospy.loginfo('Object Recognition Node Ready')
        # for test
        rospy.init_node('objectRecognition', anonymous=True)
        
        objectRecognition()
    except rospy.ROSInterruptException:
        rospy.logerror('Error ROS')
