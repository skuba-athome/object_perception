import os
import random
from subprocess import call
import sys

startPath = "PicCut/"
prefixFolder = "LocalizationTrain/"

if not os.path.exists(prefixFolder):
    os.makedirs(prefixFolder)

#startPath = "PicCut/"

index = sys.argv[1]
nnn = int(sys.argv[2])

# Extract SURF keypoints
Picture = []

objectIndex = 0
for folder in os.listdir(startPath):
    Picture.append([])
    objectIndex += 1
    for picture in os.listdir(startPath + folder + "/"):
        if picture.endswith(".jpg"):
			filePath = startPath + folder + "/" + picture
			surfPath,extension = os.path.splitext(filePath)
			Picture[objectIndex-1].append((surfPath+".txt",objectIndex))
			#Picture.append((surfPath+".txt",objectIndex))
			print filePath
			#call(["./extractSURF",filePath,surfPath+".txt"])
			call(["./extractSURF",filePath,surfPath+".txt"])

trainList = Picture[:-50]
testingList = Picture[-50:]
trainList = []
testingList = []
for i in range(len(Picture)):
	random.shuffle(Picture[i])
	trainList += Picture[i][:-nnn]
	testingList += Picture[i][-nnn:]

print len(trainList),len(testingList)

# create file train

fileTrain = open(prefixFolder + index + ".train","w")
for aObject in trainList:
    fileTrain.write(str(aObject[1])+" "+aObject[0]+"\n")
fileTrain.close()

# create file testing

fileTrain = open(prefixFolder + index +".test","w")
for aObject in testingList:
	fileTrain.write(str(aObject[1])+" "+aObject[0]+"\n")
fileTrain.close()

#call(["./writeObjectOrder.py"])
