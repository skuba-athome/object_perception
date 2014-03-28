from sklearn import neighbors, datasets
import roslib
roslib.load_manifest('object_perception')
import sys
import os
import cv2 
import rospy
import time 
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from object_perception.msg import cropped_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

publisher = rospy.Publisher('verified_result', String)

prefixInput = ""
weights = ""
features = []
labels = []
n_neighbors = 0
clf = None

def loadTestPicture(fileName):
   filePtr = open(fileName,"r")
   picture_list = []
   for line in filePtr:
	   tempStr = line.strip().split(" ")
	   fileFeature = open("/home/skuba/skuba_athome/object_perception/learn/"+tempStr[1],"r")
	   feature_list = []
	   label_list = []
	   for aLine in fileFeature:
		   attribute = aLine.strip().split(",")
		   feature_list.append(map(float,attribute))
		   label_list.append(int(tempStr[0]))
	   fileFeature.close()
	   picture_list.append((label_list,feature_list))
   filePtr.close()
   return picture_list
 
def loadData(fileName):
    pictureData = loadTestPicture(fileName) 
    feature = []
    label = []
    for aPicture in pictureData:
	aLabel,aFeature = aPicture
        feature += aFeature
        label += aLabel
    return feature,label

def NaiveFunction2(clf,result,feature):
	global labels
	dis,ind = clf.kneighbors(feature)
	classSet = []
	aRes = [0.0 for x in result]
	for x in range(len(ind[0])):
		if(labels[ind[0][x]] in classSet):
			continue
		classSet.append(labels[ind[0][x]])
		aRes[labels[ind[0][x]]-1] = dis[0][x] - dis[0][-1]
	return aRes

def cropped_msg_cb(data):
	global prefixInput,clf,weights,features,labels
	print 'In cropped msg cb'
	THRESOLD = -999


	print "{1} {0}-NN".format(n_neighbors,prefixInput)

	probLabel = [labels.count(i) for i in set(labels)]

	print data.filePath

	testFile = open(data.filePath,"r")
	testFeature = []
	for line in testFile:
		element = line.split(',')
		testFeature.append(map(float,element))
	if len(testFeature) == 0 :
		publisher.publish("-1 " + str(data.vector.x) + " " + str(data.vector.y))
		print("-1 " + str(data.vector.x) + " " + str(data.vector.y))
		return 0
		
	print "len of each feature : " + str(len(testFeature[0]))
			

	imgFeature = testFeature
	result = [0.0 for i in probLabel]

	objectClass =[0 for i in probLabel]

	for aFeature in imgFeature:
		aRes = NaiveFunction2(clf,result,aFeature)
		for x in range(len(aRes)):
			result[x] += aRes[x]


	labelResult = ''
	firstMin = min(result);
	firstLabelObject = result.index(min(result)) + 1

	result[result.index(min(result))] = 999
	secondMin = min(result);
	secondLabelObject = result.index(min(result)) + 1
	
	if(abs(firstMin - secondMin) < THRESOLD):
		labelResult = -1
	else:
		labelResult = firstLabelObject

	print str(labelResult) + " " + str(data.vector.x) + " " + str(data.vector.y)

	publisher.publish(str(labelResult) + " " + str(data.vector.x) + " " + str(data.vector.y))

def main():

	global prefixInput,clf,weights,n_neighbors,labels,features
	rospy.Subscriber("cropped_msg",cropped_msg,cropped_msg_cb)
	rospy.init_node('Naivebayes', anonymous=True)

	prefixInput = "/home/skuba/skuba_athome/object_perception/learn/LocalizationTrain/15.train"
	features,labels = loadData(prefixInput)
	weights = 'distance'
	n_neighbors = 20
	clf = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
	clf.fit(features, labels) 

	rospy.loginfo('Verification Start')
	rospy.spin()

if __name__ == '__main__':
    try:
	rospy.loginfo('Verification Node Start Initializing')
        main()
    except rospy.ROSInterruptException:
        pass
