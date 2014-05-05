#!/usr/bin/env python
import roslib
import rospy
import cv2
import os
import sys
from subprocess import call
import numpy
import time
import math
import random

roslib.load_manifest('object_recognition')
from object_recognition.srv import *
from std_msgs.msg import String
#from object_recognition.objectRecognitionNaiveBayes import *
#from objectRecognitionNaiveBayes import objectRecognition
from sklearn import neighbors, datasets

from operator import add
object_dic = []
surf = cv2.SURF(400)

class extract_folder:
    def __init__(self):
        self.startPath = "/home/skuba/webcam_data_640x480/cropped/"
        #startPath = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/PicCut/"
        #prefixFolder = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/"
        self.prefixFolder = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/"
        self.featurePath = "/home/skuba/skuba_athome/object_perception/object_recognition/config/feature/"

    def extract_to_config(self):
        if not os.path.exists(self.featurePath):
            os.makedirs(self.featurePath)

        objectIndex = 0
        for folder in os.listdir(self.startPath):

            #print folder
            #Picture.append([])
            objectIndex += 1
            surfPath = self.featurePath + folder + ".txt"
            if os.path.exists(surfPath):
                os.remove(surfPath)

            #print surfPath
            for picture in os.listdir(self.startPath + folder + "/"):
                if picture.endswith(".jpg") or picture.endswith(".png"):
                    #print picture
                    filePath = self.startPath + folder + "/" + picture

#//                if os.path.exists(filePath):
#//                    os.remove(filePath)
                    print 'in extract_to_config method'
                    call(["/home/skuba/skuba_athome/object_perception/object_recognition/bin/extractSURF",filePath,surfPath])


    def extract_to_each_picture(self):
        startPath = "/home/skuba/webcam_data_640x480/cropped/"
        prefixFolder = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/"

        if not os.path.exists(prefixFolder):
            os.makedirs(prefixFolder)
        index = 15
        nnn = 1

        # Extract SURF keypoints
        Picture = []

        objectIndex = 0
        print os.listdir(startPath)
        for folder in os.listdir(startPath):
            Picture.append([])
            objectIndex += 1
            print folder
            for picture in os.listdir(startPath + folder + "/"):
                if picture.endswith(".txt"):
                    os.remove(startPath + folder + "/" + picture)

            for picture in os.listdir(startPath + folder + "/"):
                if picture.endswith(".jpg") or picture.endswith(".png"):
                    filePath = startPath + folder + "/" + picture
                    surfPath,extension = os.path.splitext(filePath)
                    print '----',filePath,'----\n'
                    #print os.path.splitext(filePath)
                    #Picture[objectIndex-1].append((surfPath+".txt",objectIndex))
                    Picture[objectIndex-1].append((surfPath+".txt",folder))
                    print 'in extract_to_each_picture method'
                    call(["/home/skuba/skuba_athome/object_perception/object_recognition/bin/extractSURF",filePath,surfPath+".txt"])

        #trainList = Picture[::]
##        trainList = Picture[:-50]
##        testingList = Picture[-50:]
        trainList = []
        testingList = []
        for i in range(len(Picture)):
            random.shuffle(Picture[i])
            trainList += Picture[i][::]
            #trainList += Picture[i][:-nnn]
            #testingList += Picture[i][-nnn:]
#
#        print len(trainList),len(testingList)
#
#
#
#        # create file train
#
        fileTrain = open(prefixFolder + str(index) + ".train","w")
        for aObject in trainList:
            fileTrain.write(str(aObject[1])+","+str(aObject[0])+"\n")
            print (str(aObject[1])+","+str(aObject[0])+"\n")
        fileTrain.close()

        # create file testing
#
#        fileTrain = open(prefixFolder + str(index) +".test","w")
#        for aObject in testingList:
#            fileTrain.write(str(aObject[1])+" "+str(aObject[0])+"\n")
#        fileTrain.close()

class find_variable_threshold:
    def __init__(self):
        print 'in find_variable_threshold init0'
        self.classify_object = rospy.ServiceProxy('classifyObject', classifyObject)
        self.object_recognition = objectRecognition()
        print 'in find_variable_threshold init1'
        self.k_fold = rospy.get_param('~k_fold', 10)
        self.object_root_dir = rospy.get_param('~object_directory',"/home/skuba/webcam_data_640x480/cropped/")
        self.train_file = []
        self.test_file = {}
        self.object_dic = self.list_image_in_directory(self.object_root_dir)
        #print self.object_dic
        self.result = {}
        for file_name in self.object_dic:
            self.result[file_name] = {}
            self.result[file_name]["correct"] = []
            self.result[file_name]["wrong"] = []
        self.iteration = 1
        self.percentile = 0.95
        self.object_threshold = {}
        #time.sleep(3)
        self.recreate_location = '/home/skuba/skuba_athome/object_perception/object_recognition/learn/file_location.txt'
        print 'Initialization complete with k_fold : %d.'%(self.k_fold)
        rospy.loginfo('find_variable_threshold Start')



    def list_image_in_directory(self,dir):
        pic_dic = {}
        for object_dir in os.listdir(dir):
            object_dir_path = os.path.join(dir, object_dir)
            if not os.path.isdir(object_dir_path):
                continue
            pic_dic[str(object_dir)] = []
            for object_pic in os.listdir(object_dir_path):
                if object_pic.endswith(".png") or object_pic.endswith(".jpg"):
                    pic_dic[str(object_dir)].append(str(os.path.join(object_dir_path, object_pic)))
        return pic_dic

    def run(self):
        #self.k_fold_validation()
        self.k_attempt_validation()
        print 'after k_attempt_validation'
        f = open("/home/skuba/.ros/find_variable_threshold.txt",'w')
        threshold_writer = open("/home/skuba/skuba_athome/object_perception/object_recognition/config/result_threshold.txt",'w')
        for object_name in self.object_dic:
            if len(self.result[object_name]["correct"]) != 0 :
                correct_tmp = reduce(lambda x,y : x+y , self.result[object_name]["correct"])
                if len(correct_tmp) != 0:
                    #correct_max = float(sum(correct_tmp))/len(correct_tmp)
                    correct_tmp = sorted(correct_tmp)
                    correct_max = correct_tmp[-1]#max(correct_tmp)
                    self.object_threshold[object_name] = correct_tmp[int(len(correct_tmp)*self.percentile)]
                    #self.object_threshold[object_name] = correct_max
                    print self.object_threshold[object_name] 
                    f.write(object_name + '(correct) : '+ str(correct_max) +'\n'+ str(reduce(lambda x,y : "%s\n%s"%(str(x),str(y)) ,correct_tmp)) +'\n') 
                    threshold_writer.write(str(object_name) + ":" + str(self.object_threshold[object_name]) + '\n')
                else:
                    f.write(object_name + '(correct) : '+ str(0) +'\n'+'no sequence\n') 

            else:
                f.write(object_name + '(correct) : '+ str(0) +'\n'+'no sequence\n') 

            print 'self.result[object_name]["wrong"]',self.result[object_name]["wrong"]
            wrong_max = 0
            if len(self.result[object_name]["wrong"]) != 0 :
                wrong_tmp = reduce(lambda x,y : x+y , self.result[object_name]["wrong"])
                print 'len(wrong_tmp)',len(wrong_tmp),str(wrong_tmp)
                if len(wrong_tmp) != 0:
                    #wrong_avg = float(sum(wrong_tmp))/len(wrong_tmp)
                    wrong_max = max(wrong_tmp)
                    f.write(object_name + '(wrong) : '+ str(wrong_max) +'\n'+ str(reduce(lambda x,y : "%s\n%s"%(str(x),str(y)) ,wrong_tmp)) +'\n') 
                else:
                    f.write(object_name + '(wrong) : '+ str(0) +'\n'+'no sequence\n') 
            else:
                f.write(object_name + '(wrong) : '+ str(0) +'\n'+'no sequence\n') 


            print 'average correct confident of',object_name,'=',correct_max
            print 'average wrong confident of',object_name,'=',wrong_max
        f.close()

    def train_model(self,file_list):
        self.write_location_file(file_list)
        self.object_recognition.create_model(self.recreate_location)
        #self.pub.publish(self.recreate_location)

    def write_location_file(self,file_list):
        f = open(self.recreate_location,'w')
        for line in file_list:
            f.write(line+'\n') 
        f.close()

    def validate(self,test_file):
        print "iteration",self.iteration
        self.iteration+=1
        f = open('/home/skuba/.ros/wrong_prediction.txt','a')
        for object_name in self.object_dic:
            correct_prediction = []
            wrong_prediction = []
            #print 'object_name',object_name
            for file_name in test_file[object_name]:
                category,confident= self.object_recognition.predictObject(file_name)
                if category == object_name:
                    correct_prediction.append(confident)
                else:
                    wrong_prediction.append(confident)
                    print 'wrong prediction at',file_name,'predict as',category
                    f.write("predict : "+category+" from : "+file_name+'\n') 
            self.result[object_name]["correct"].append(correct_prediction)
            self.result[object_name]["wrong"].append(wrong_prediction)
        f.close()

    def k_attempt_validation(self):
        test_ratio = 0.2
        for i in range(self.k_fold):
            test_file = {}
            train_file = []
            for object_name in self.object_dic:
                testing_number = int(len(self.object_dic[object_name])*test_ratio)
                #number_per_round = len(self.object_dic[object_name])/self.k_fold
                random.shuffle(self.object_dic[object_name])
                test_file[object_name] = self.object_dic[object_name][:testing_number]
                train_tmp = self.object_dic[object_name][testing_number:]
                #test_file[object_name] = self.object_dic[object_name][i*number_per_round:(i+1)*number_per_round]
                #train_tmp = (self.object_dic[object_name][:i*2] + self.object_dic[object_name][(i+1)*number_per_round:])
                #train_file_prefixed = map(lambda x : "%s %s"%(object_name,x),train_tmp)
                train_file += map(lambda x : "%s,%stxt"%(object_name,x[:-3]),train_tmp)
            print train_file
            self.train_model(train_file)
            self.validate(test_file)


    def k_fold_validation(self):
        for i in range(self.k_fold):
            test_file = {}
            train_file = []
            for object_name in self.object_dic:
                number_per_round = len(self.object_dic[object_name])/self.k_fold
                test_file[object_name] = self.object_dic[object_name][i*number_per_round:(i+1)*number_per_round]
                train_tmp = (self.object_dic[object_name][:i*2] + self.object_dic[object_name][(i+1)*number_per_round:])
                #train_file_prefixed = map(lambda x : "%s %s"%(object_name,x),train_tmp)
                train_file += map(lambda x : "%s %stxt"%(object_name,x[:-3]),train_tmp)

            self.train_model(train_file)
            self.validate(test_file)

    def call_for_service(self,file_name):
        rospy.wait_for_service('classifyObject')
        try:
            resp = self.classify_object(file_name)
            return resp.objectCategory, resp.confident
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

class objectRecognition:
    def __init__(self):
        rospy.Subscriber("featureFilePath",String,self.classifyObjectHandle)
        self.recognitionResult = rospy.Publisher('recognitionResult',String) 
        print 'in objectRecognition init'
        learningListFile = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/15.train"
        feature,self.labels = self.loadTrainData(learningListFile)
        print 'in objectRecognition init2'
        self.K_neighbors = int(rospy.get_param('~k_neighbors', 35))
        self.clf = neighbors.KNeighborsClassifier(self.K_neighbors, weights='distance')
        self.clf.fit(feature, self.labels) 
        
        self.object_threshold_file = "/home/skuba/.ros/result_threshold.txt"
        self.object_threshold = {}
#        self.object_threshold = self.read_object_threshold(self.object_threshold_file)
#        print self.object_threshold
        rospy.Service('classifyObject',classifyObject,self.classifyObjectService)
        
        rospy.loginfo('Verification Start')
        #rospy.spin()
		

    def read_object_threshold(self,file_name):
        filePtr = open(file_name,"r")
        threshold_list = {}
        for line in filePtr:
            object_name, threshold = line.split()
            threshold_list[object_name] = float(threshold)
        return threshold_list

    def loadTrainData(self,filename):
        self.categorySet = {}
        self.revertCategory = []
        feature_list = []
        label_list = []
        #print filename
        filePtr = open(filename,"r")
        for line in filePtr:
            #print line
            #category,filePath = line.strip().split(",")
            category,filePath = line.strip().split(",")
            if not category in self.categorySet:
                self.categorySet[category] = len(self.categorySet)
                self.revertCategory.append(category)
            #print '--',filePath,self.categorySet[category]
            feature,label = self.loadFeature(filePath,self.categorySet[category])
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
    
    def create_model(self,file_name):
        feature,self.labels = self.loadTrainData(file_name)
        self.clf = neighbors.KNeighborsClassifier(self.K_neighbors, weights='distance')
        self.clf.fit(feature, self.labels) 

    def classifyObjectHandle(self,data):
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

        #print len(keypoint)

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
        #print "filename :",featureFileName , "with difference :",str(sortedWeightSum[0]-sortedWeightSum[1])
        distance = sortedWeightSum[0]-sortedWeightSum[1]
#        if self.object_threshold[str(self.revertCategory[int(weightSum.index(min(weightSum)))])] < distance:
#            return (str(self.revertCategory[int(weightSum.index(min(weightSum)))]),distance)
#        return 'unknown',(distance)
        return str(self.revertCategory[int(weightSum.index(min(weightSum)))]), (sortedWeightSum[0]-sortedWeightSum[1])
    
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

if __name__ == "__main__":
    try:
        rospy.init_node('find_variable_threshold')
        ef = extract_folder()
        ef.extract_to_config()
        ef.extract_to_each_picture()
        print 'after extraction finished'
        fvt = find_variable_threshold()
        fvt.run()
        rospy.spin()
    except Exception, error:
        print str(error)
