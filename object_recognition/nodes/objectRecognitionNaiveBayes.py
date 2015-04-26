#!/usr/bin/python
from sklearn import neighbors, datasets
from sklearn.externals import joblib
from sklearn.svm import SVC
import cv2
import os
import numpy
import roslib
import rospy

from operator import add
from std_msgs.msg import String
from object_recognition_new.srv import *

#svm_model_filename = 'svm_model.pkl'
object_root_dir = "/home/mukda/object_recognition_new/data3/"
svm_model_filename = '/home/mukda/object_recognition_new/src/svm_model/svm_model.pkl'
surf = cv2.SURF(400)

class objectRecognition:

    def __init__(self):
        rospy.init_node('ObjectRecognitionNaivebayes', anonymous=True)
        #rospy.Subscriber("featureFilePath",String,self.classifyObjectHandle)
        #self.recognitionResult = rospy.Publisher('recognitionResult',String) 
        
        object_dic = self.list_image_in_directory(object_root_dir,'/test/')
        feature, self.labels = self.loadTrainData(object_dic)

        K_neighbors = int(rospy.get_param('~k_neighbors', 35))
        #K_neighbors = 35
        self.clf = neighbors.KNeighborsClassifier(K_neighbors, weights='distance')
        self.clf.fit(feature, self.labels) 
        
        #cate, diff = self.predictObject("/home/mukda/object_recognition_new/data/minute_maid/frame0032.png")
        self.clf_svm = joblib.load(svm_model_filename)

        #test
        '''
        cate = 0
        test_dic = self.list_image_in_directory(object_root_dir,'/train/')
        for object_name in test_dic:
            count = 0
            count2 = 0
            #print object_name, cate
            
            for aImage in test_dic[object_name]:
                nb_answer, confident, queryWeightSum = self.predictObject(aImage)
                svm_answer = self.clf_svm.predict(queryWeightSum)
                if nb_answer==cate:                        
                    count+=1
                if svm_answer==cate: 
                    count2+=1
                #print aImage, nb_answer, svm_answer
                print "check cate", svm_answer, self.revertCategory[svm_answer]
                #print queryWeightSum
            print object_name, cate, "\tTotal:", len(test_dic[object_name]), "\tnb:", count, "\tsvm:", count2
            cate+=1
        '''

        rospy.Service('classifyObject', classifyObject, self.classifyObjectService)

        rospy.loginfo('Verification Start')
        rospy.spin()            


    def list_image_in_directory(self, dir, name):
        image_dic = {}
        for object_dir in os.listdir(dir):
            object_dir_path = os.path.join(dir, object_dir) + name
            #object_dir_path = os.path.join(dir, object_dir) + '/train/'
            #object_dir_path = os.path.join(dir, object_dir)
            if not os.path.isdir(object_dir_path):
                continue
            image_dic[str(object_dir)] = []
            for object_pic in os.listdir(object_dir_path):
                if object_pic.endswith(".jpg") or object_pic.endswith(".png"):
                    image_dic[str(object_dir)].append(str(os.path.join(object_dir_path, object_pic)))
        return image_dic


    def loadTrainData(self, object_dic):
    #def loadTrainData(self,dir_name):
        self.categorySet = {}       #dict
        self.revertCategory = []    #array
        feature_list = []
        label_list = []


        #for object_dir in os.listdir(dir_name):
        for object_name in object_dic:
            #file_name = dir_name + object_dir + "/features.txt"
            file_name = object_root_dir + object_name + "/features.txt"
            #print file_name
            #category = object_dir
            category = object_name

            if not category in self.categorySet:
                self.categorySet[category] = len(self.categorySet)
                self.revertCategory.append(category)

            feature,label = self.loadFeature(file_name, self.categorySet[category])
            feature_list += feature
            label_list += label
            #print object_dir, self.categorySet[category]
            #print object_name, self.categorySet[category]
        return feature_list,label_list


    def loadFeature(self,filename,category):
        feature_list = []
        label_list = []
        filePtr = open(filename,"r")
        
        for line in filePtr:
            attribute = line.strip().split(" ")
            #attribute = line.strip().split(",")
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
        #category,confident = self.predictObject(req.filepath)
        nb_answer, confident, queryWeightSum = self.predictObject(req.filepath)
        svm_answer = self.clf_svm.predict(queryWeightSum)
        print str(self.revertCategory[nb_answer]),confident
        #return classifyObjectResponse(str(self.revertCategory[category]),confident)
        return classifyObjectResponse(str(self.revertCategory[nb_answer]),confident)
        #return classifyObjectResponse(str(self.revertCategory[svm_answer]),confident)

#        if self.object_threshold[str(self.revertCategory[category])] < confident:
#            return classifyObjectResponse(str(self.revertCategory[category]),confident)
#        return classifyObjectResponse("unknown",confident)
        
    # this is Main function
    def predictObject(self,featureFileName):
        image = cv2.imread(featureFileName, 0)
        queryFeature = []
        keypoint, features = surf.detectAndCompute(image, None)        

        if features == None or len(features) == 0:
            return -1

        for feature in features:
            queryFeature.append(map(float,feature))

        weightSum = [0.0 for i in self.categorySet]
        for aFeature in queryFeature:
            weight = self.predictFeature(aFeature)
            weightSum = map(add, weight, weightSum)
        

        sortedWeightSum = sorted(weightSum)
        nb_answer = int(weightSum.index(min(weightSum)))
        confident = sortedWeightSum[0]-sortedWeightSum[1]
        #print "\nfilename :",featureFileName ,int(weightSum.index(min(weightSum)))
        #return weightSum
        return nb_answer, confident, weightSum
        #print "  with difference :",str(sortedWeightSum[0]-sortedWeightSum[1])
        #return int(weightSum.index(min(weightSum))), 
        

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
        rospy.loginfo('Verification Node Start Initializing')
        objectRecognition()
    except rospy.ROSInterruptException:
        rospy.logerror('Error ROS')