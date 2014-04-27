#!/usr/bin/env python
import roslib
import rospy
import cv2
import os
import numpy

roslib.load_manifest('object_recognition')
from object_recognition.srv import *

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
    rospy.wait_for_service('/object_recognition/verify_object')

    #verify_object = rospy.ServiceProxy('/object_recognition/verify_object', verifyObject)
    #verify_object = rospy.ServiceProxy('/object_recognition/verify_object',classify_object_service)
    verify_object = rospy.ServiceProxy('/object_recognition/verify_object',classifyObject)

    #rospy.Service('/object_recognition/verify_object', classifyObject, self.classify_object_service)



    #verify_object = rospy.ServiceProxy('verifyObject', verifyObject)
    #object_root_dir = rospy.get_param('~object_directory', roslib.packages.get_pkg_dir('object_recognition') + '/data')
    object_root_dir = rospy.get_param('~object_directory',"/home/skuba/test_data/")
#    print object_root_dir
    object_dic = list_image_in_directory(object_root_dir)
    print object_dic

    #f = open('/home/skuba/.ros/result','w')
    print object_root_dir
    for directory_path in object_dic:
        for file_name in object_dic[directory_path]:
            print file_name
            response = verify_object(file_name)
            print response
