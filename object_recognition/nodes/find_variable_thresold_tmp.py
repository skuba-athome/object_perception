#!/usr/bin/env python
import roslib
import rospy
import cv2
import os
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

test_features = None
test_labels = None

class find_variable_threshold:
    def __init__(self):
        self.classify_object = rospy.ServiceProxy('classifyObject', classifyObject)
        self.object_recognition = objectRecognition()
        self.k_fold = rospy.get_param('~k_fold', 2)
        #self.object_root_dir = rospy.get_param('~object_directory',"/home/skuba/skuba_athome/object_perception/object_recognition/config/feature/")
        self.object_root_dir = rospy.get_param('~object_directory',"/home/skuba/webcam_data_640x480/cropped/")
        self.train_file = []
        self.test_file = {}
        self.object_dic = self.list_image_in_directory(self.object_root_dir)

#        for object_name in self.object_dic:
#            for tmp in self.object_dic[object_name]:
#                print tmp
#
        #print self.object_dic

        self.result = {}
        for file_name in self.object_dic:
            self.result[file_name] = {}
            self.result[file_name]["correct"] = []
            self.result[file_name]["wrong"] = []
        self.iteration = 1
        self.percentile = 0.9
        self.object_threshold = {}
        #time.sleep(3)
        self.recreate_location = '/home/skuba/skuba_athome/object_perception/object_recognition/learn/file_location.txt'
        print 'Initialization complete with k_fold : %d.'%(self.k_fold)
        rospy.loginfo('find_variable_threshold Start')



    def list_image_in_directory(self,dir):
        pic_dic = {}
        for object_dir in os.listdir(dir):
            object_dir_path = os.path.join(dir, object_dir)
            print object_dir_path
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

    def train_model(self):
    #def train_model(self,file_list):
        #self.write_location_file(file_list)
        self.object_recognition.create_model_new(5)
        #self.pub.publish(self.recreate_location)

    def write_location_file(self,file_list):
        f = open(self.recreate_location,'w')
        for line in file_list:
            f.write(line+'\n') 
        f.close()

    #def validate(self,test_file):
    def validate(self):

        global test_features,test_labels
        print "iteration",self.iteration
        self.iteration+=1
        f = open('/home/skuba/.ros/wrong_prediction.txt','a')
#        for object_name in self.object_dic:
#            correct_prediction = []
#            wrong_prediction = []
            #print 'object_name',object_name

        print 'len(test_features)',len(test_features)
#        for i in range(len(test_features)):
#            print type(test_features[i])
#        print '-----------------------------------------------------------'

#        for tmp in test_features[0]:
#            print type(tmp)
    
        for object_number in range(len(test_features)):
            for i in range(len(test_features[object_number])):
                category,confident = self.object_recognition.predictObject_new(test_features[object_number][i],test_labels[object_number])
                print '------------',category,confident
                correct_prediction = []
                wrong_prediction = []

                print category,test_labels[object_number]
                if category == test_labels[object_number]:
                    print 'right prediction with confident',confident
                    correct_prediction.append(confident)
                else:
                    wrong_prediction.append(confident)
                    print 'wrong prediction at',str(test_labels[object_number]),'predict as',category
                    f.write("predict : "+category+" from : "+str(test_labels[object_number])+'\n') 
                self.result[test_labels[i]]["correct"].append(correct_prediction)
                self.result[test_labels[i]]["wrong"].append(wrong_prediction)

            #for file_name in test_file[object_name]:
            #    category,confident= self.object_recognition.predictObject(file_name)
            #    if category == object_name:
            #        correct_prediction.append(confident)
            #    else:
            #        wrong_prediction.append(confident)
            #        print 'wrong prediction at',file_name,'predict as',category
            #        f.write("predict : "+category+" from : "+file_name+'\n') 
            #self.result[object_name]["correct"].append(correct_prediction)
            #self.result[object_name]["wrong"].append(wrong_prediction)
        f.close()
        print 'end validate method'

    def k_attempt_validation(self):
        test_ratio = 0.2
        for i in range(self.k_fold):
#            test_file = {}
#            train_file = []
#            for object_name in self.object_dic:
#                testing_number = int(len(self.object_dic[object_name])*test_ratio)
#                #number_per_round = len(self.object_dic[object_name])/self.k_fold
#                random.shuffle(self.object_dic[object_name])
#                test_file[object_name] = self.object_dic[object_name][:testing_number]
#                train_tmp = self.object_dic[object_name][testing_number:]
#                #test_file[object_name] = self.object_dic[object_name][i*number_per_round:(i+1)*number_per_round]
#                #train_tmp = (self.object_dic[object_name][:i*2] + self.object_dic[object_name][(i+1)*number_per_round:])
#                #train_file_prefixed = map(lambda x : "%s %s"%(object_name,x),train_tmp)
#                train_file += map(lambda x : "%s %stxt"%(object_name,x[:-3]),train_tmp)
            self.train_model()
            #self.validate(test_file)
            self.validate()


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
        self.revertCategory = {}
        
        #learningListFile = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/15.train"
        feature_directory = "/home/skuba/skuba_athome/object_perception/object_recognition/config/feature/"
        #feature,self.labels = self.loadTrainData(learningListFile)
        self.feature, self.labels = self.load_train_data(feature_directory)

        data = map(lambda x,y : "%s %s"%(x,reduce((lambda x,y : "%s,%s"%(x,y)),y)),self.labels,self.feature)
        self.train_list = {}

        #print type(revertCategory)

        for line in data:
            label,features = line.split(' ')
            if not self.train_list.has_key(label):
                self.train_list[label] = []
            features = map(float,features.split(','))
            self.train_list[label].append(features)

        self.K_neighbors = int(rospy.get_param('~k_neighbors', 35))
        self.clf = neighbors.KNeighborsClassifier(self.K_neighbors, weights='distance')
        self.clf.fit(self.feature, self.labels) 
        
        self.object_threshold_file = "/home/skuba/.ros/result_threshold.txt"
        self.object_threshold = {}
#        self.object_threshold = self.read_object_threshold(self.object_threshold_file)
#        print self.object_threshold
        rospy.Service('classifyObject',classifyObject,self.classifyObjectService)
        
        rospy.loginfo('Verification Start')
        #rospy.spin()
		


    def load_train_data(self, feature_directory):
        feature_list = []
        label_list = []
        self.category_set = []
        self.categorySet = {}
        index = 0
        for feature_file in os.listdir(feature_directory):
            if feature_file.endswith(".txt"):
                category = os.path.splitext(os.path.basename(feature_file))[0]
                self.category_set.append(category)
                category_index = self.category_set.index(category)
                feature, label = self.load_feature(os.path.join(feature_directory,feature_file), category_index)

                if not category_index in self.categorySet:
                    self.categorySet[category_index] = len(self.categorySet)

                self.revertCategory[index] = feature_file[:feature_file.index('.txt')]
                #print self.revertCategory[index] 
                feature_list += feature
                label_list += label
                index+=1
        self.category_set.append('unknown')
        #print 'end of load_train_data'
        self.revertCategory[index] = 'unknown'
        return feature_list,label_list


    def load_feature(self, filename, category):
        #print 'in load_feature : ',filename,category
        feature_list = []
        label_list = []
        file_feature = open(filename,"r")
        for line in file_feature:
            attribute = line.strip().split()
            feature_list.append(map(float,attribute))
            label_list.append(int(category))
        file_feature.close()
        return feature_list,label_list


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
            category,filePath = line.strip().split(" ")
            #print filePath
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
    
    def create_model(self,file_name):
        print 'bug here'
        #feature,self.labels = self.loadTrainData(file_name)
        self.clf = neighbors.KNeighborsClassifier(self.K_neighbors, weights='distance')
        #self.clf.fit(feature, self.labels) 
        self.clf.fit(train_feature, train_labels) 
        print 'bug here'


    def create_model_new(self,index):
        global test_features,test_labels
        train_feature = []
        train_label = []

        test_features = []
        test_labels = []

        for object_name in self.train_list:
#            print '--------',object_name,self.revertCategory[int(object_name)]
#            for tmp in self.train_list[object_name]:
                #print tmp
            #print '----------',object_name,self.revertCategory[object_name]
            print 'in create_model_new',object_name

            #random.shuffle(self.train_list[object_name])
            test_features.append(self.train_list[object_name][:index])
            #test_labels.append([self.revertCategory[int(object_name)] for i in range(len(self.train_list[object_name][:index]))])
            test_labels.append(object_name)
            #test_labels.append(object_name for i in range(len(self.train_list[object_name][:index])))

            train_feature += self.train_list[object_name][index:]
            train_label += [object_name for i in range(len(self.train_list[object_name][index:]))]
        print 'len(test_features)',len(test_features)
#        for i in test_features:
#            print type(i),type(i[0]),type(i[0][0])
        #print len(train_label)

        #feature,self.labels = self.loadTrainData(file_name)
        self.clf = neighbors.KNeighborsClassifier(self.K_neighbors, weights='distance')
        self.clf.fit(train_feature, train_label) 
        #feature,self.labels = self.loadTrainData(file_name)
        #self.clf.fit(train_feature, train_labels) 

    def classifyObjectHandle(self,data):
        category = self.predictObject(data.data)
        print String(self.revertCategory[category])
        self.recognitionResult.publish(String(self.revertCategory[category]))
    
    def classifyObjectService(self,req):
        print "incoming input :",req.filepath
        category,confident = self.predictObject(req.filepath)
        return classifyObjectResponse(str(self.revertCategory[category]),confident)



    def predictObject_new(self,features,keypoint):
        queryFeature = []
        if features == None or len(features) == 0:
            return -1

        print type(features)
        print type(features[0])
        for feature in features:
            #queryFeature.append(map(float,feature))
            queryFeature.append(map(float,feature))
        #queryFeature = features

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
        return str(int(weightSum.index(min(weightSum)))), (sortedWeightSum[0]-sortedWeightSum[1])
    

    
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

        tmp_ = [0.0 for i in self.categorySet]
        print 'tmp : ',tmp_
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
        fvt = find_variable_threshold()
        fvt.run()
        rospy.spin()
    except Exception, error:
        print str(error)
