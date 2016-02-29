#!/usr/bin/env python
import os
import random
from subprocess import call
import sys

startPath = "/home/skuba/webcam_data_640x480/cropped/"
#startPath = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/PicCut/"
#prefixFolder = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/"
prefixFolder = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/LocalizationTrain/"
featurePath = "/home/skuba/skuba_athome/object_perception/object_recognition/config/feature/"

#if not os.path.exists(prefixFolder):
#    os.makedirs(prefixFolder)

# Extract SURF keypoints
Picture = []

objectIndex = 0
for folder in os.listdir(startPath):
    #print folder
    Picture.append([])
    objectIndex += 1
    surfPath = featurePath + folder + ".txt"
    if os.path.exists(surfPath):
        os.remove(surfPath)
    for picture in os.listdir(startPath + folder + "/"):
        if picture.endswith(".jpg") or picture.endswith(".png"):
            print picture
            filePath = startPath + folder + "/" + picture
            #surfPath,extension = os.path.splitext(filePath)

            #Picture[objectIndex-1].append((surfPath+".txt",objectIndex))
            #Picture[objectIndex-1].append((surfPath+".txt",folder))
            #Picture.append((surfPath+".txt",objectIndex))
            #print filePath," ",surfPath
            call(["/home/skuba/skuba_athome/object_perception/object_recognition/bin/extractSURF",filePath,surfPath])
            #call(["./extractSURF",filePath,surfPath+".txt"])
