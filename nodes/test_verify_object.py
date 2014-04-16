#!/usr/bin/env python
import roslib
import rospy
import cv2
import os
import numpy

roslib.load_manifest('object_perception')
from object_perception.srv import *

object_dic = []

def list_image_in_directory(dir):
    pic_dic = {}
    for object_dir in os.listdir(dir):
        #object_dir_path = os.path.join(dir, object_dir) + '/test/'
        object_dir_path = os.path.join(dir, object_dir)
        if not os.path.isdir(object_dir_path):
            continue
        pic_dic[str(object_dir)] = []
        for object_pic in os.listdir(object_dir_path):
            #print os.path.join(object_dir_path, object_pic)
            if object_pic.endswith(".jpg") or object_pic.endswith(".png"):
                pic_dic[str(object_dir)].append(str(os.path.join(object_dir_path, object_pic)))
    return pic_dic

if __name__ == "__main__":
    rospy.wait_for_service('verifyObject')

    verify_object = rospy.ServiceProxy('verifyObject', verifyObject)
    #object_root_dir = rospy.get_param('~object_directory', roslib.packages.get_pkg_dir('object_perception') + '/data')
    object_root_dir = rospy.get_param('~object_directory',"/home/skuba/new_data/")
    print object_root_dir
    object_dic = list_image_in_directory(object_root_dir)

    f = open('/home/skuba/.ros/result','w')
    print object_root_dir
    for directory_path in object_dic:
        #f.write(directory_path+'\n') 
        accuracy = 0
        if len(object_dic[directory_path]) == 0:
            continue
        for file_name in object_dic[directory_path]:
            response = verify_object(file_name)
            if response.objectName in file_name:
                accuracy+=1
        print float(accuracy)/len(object_dic[directory_path])
        f.write(directory_path + '| correct '+ str(accuracy) +'| all ' + str(len(object_dic[directory_path])) + '| accuracy' + str(float(accuracy)/len(object_dic[directory_path])) +'\n') 
    f.close() 

    #print sum([len(object_dic[object_name]) for object_name in object_dic]),"pictures found."
    # extract SURF feature
#    for object_name in object_dic:
#        extract_feature_from_images(object_dic[object_name], object_name) 


    #print dir()
    #response = verify_object("/run/shm/test.jpg")
#    response = verify_object("/run/shm/object_perception/picture0.png")

