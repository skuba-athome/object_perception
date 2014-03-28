from sklearn import neighbors, datasets
import roslib
roslib.load_manifest('objects')
import sys
import os
import cv2
import rospy
import time 
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from objects.msg import cropped_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

publisher = rospy.Publisher('verified_result', String)


def loadTestPicture(fileName):
   filePtr = open(fileName,"r")
   picture_list = []
   for line in filePtr:
	   tempStr = line.strip().split(" ")
	   #print "../learn/"+tempStr[1]
	   fileFeature = open("../learn/"+tempStr[1],"r")
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

#print prefixInput

#features,labels = loadData(prefixInput+".train")
prefixInput = "../learn/LocalizationTrain/15.train"
features,labels = loadData(prefixInput)
#print len(features[0])

#def CalculateAccuracy(label_s,label):
    #if(len(label_s) != len(label)):
        #print "Error not equal Data!!!"
        #return
    #n = len(label)
    #count = 0
    #for i in range(len(label_s)):
        #if(label_s[i] == label[i]):
            #count += 1
    #percent = (count*1.0/n)*100.0
    #print "Accuracy = {0}% ({1}/{2})".format(percent,count,n)

#weights = 'uniform'
# we create an instance of Neighbours Classifier and fit the data.
#pictures = loadTestPicture(prefixInput+".test")

#def NaiveFunction(clf,result):
    #dis,ind = clf.kneighbors(i)
    ##print dis,ind
    #count = [0 for x in result]
    #for x in ind[0]:
        #count[labels[x]-1] += 1
    #aRes = []
    #for x in range(len(count)):
        #aRes.append((count[x]*1.0/probLabel[x])*(probLabel[x]*1.0/len(labels)))
    #return aRes


# chagnge i to the descriptor?
def NaiveFunction2(clf,result,feature):
	dis,ind = clf.kneighbors(feature)
	#print dis,ind
	classSet = []
	aRes = [0.0 for x in result]
	for x in range(len(ind[0])):
		if(labels[ind[0][x]] in classSet):
			continue
		#print "x " + str(x) + ", label : " + str(labels[ind[0][x]-1])
		classSet.append(labels[ind[0][x]])
		aRes[labels[ind[0][x]]-1] = dis[0][x] - dis[0][-1]
	return aRes

def test_input_cb(data):
	print 'In cropped msg cb'
	#time.sleep(100)

	#n_neighbors = int(sys.argv[1])
	n_neighbors = 20
#	prefixInput = "../learn/LocalizationTrain/15.train"
	THRESOLD = -999

	features,labels = loadData(prefixInput)
	weights = 'distance'

	print "{1} {0}-NN".format(n_neighbors,prefixInput)

	clf = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
	clf.fit(features, labels) 
	probLabel = [labels.count(i) for i in set(labels)]

	#print data.filePath

	#testFile = open(data.filePath,"r")
	#testFeature = []
	#for line in testFile:
	#	element = line.split(',')
	#	testFeature.append(map(float,element))
	#print "len of each feature : " + str(len(testFeature[0]))

	predictedResult = []
	difference = []
	average = []
	objectName = 'coke'
	count=0
	for fileName in os.listdir("../learn/PicCut/"+objectName+"/"):
		if fileName.endswith(".jpg"):
			count+=1
			img = cv2.imread("/home/skuba/skuba_athome/objects/learn/PicCut/"+objectName+"/"+fileName)
			#print type(img)
			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			surfDetector = cv2.FeatureDetector_create("SURF")
			surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")
			keypoints = surfDetector.detect(img)
			(keypoints, descriptors) = surfDescriptorExtractor.compute(img,keypoints)
			print len(descriptors[0])

			imgFeature = descriptors
			result = [0.0 for i in probLabel]
			#for i in len(keypoints):
				#imgFeature.append(descriptors[i])

			objectClass =[0 for i in probLabel]

			for aFeature in imgFeature:
				#print "Object :" + str(label_list[0]),
			#for i in aFeature:
				#send aFeature as params
				aRes = NaiveFunction2(clf,result,aFeature)
				for x in range(len(aRes)):
					result[x] += aRes[x]
				#objectClass[index(min(result))] += 1

			summation = 0
			print '--------------'+ fileName +'-------------------'
			for i in result:
				summation += i 
				print 'result : ' + str(i)
			summation/=len(result)

			labelResult = ''
			firstMin = min(result);
			firstLabelObject = result.index(min(result)) + 1

			result[result.index(min(result))] = 999
			secondMin = min(result);
			secondLabelObject = result.index(min(result)) + 1
			result[result.index(min(result))] = 999
			
			print 'firstMin : ' + str(firstMin) + ' secondMin : ' + str(secondMin) + ' difference : ' + str(firstMin - secondMin)

			if(abs(firstMin - secondMin) < THRESOLD):
				labelResult = -1
			else:
				labelResult = firstLabelObject

			print str(labelResult)
			predictedResult.append((firstMin,secondMin,firstMin-secondMin,labelResult,fileName))
			difference.append(abs(firstMin-secondMin))
			average.append(summation)
			publisher.publish(str(labelResult) + " " + str(data.vector.x) + " " + str(data.vector.y))

		#print 'count : ' + str(count)
		print '-------------------' + str(len(predictedResult)) + ' ' + str(len(difference)) + ' ' + str(len(average))
		print '-------------------------'
		for result in predictedResult:
			print result
		print '---------diff----------------'
		for diff in difference:
			print diff
		print '---------avg----------------'
		for avg in average:
			print avg
		print '---------max-avg----------------'

		j=0
		for result in predictedResult:
			print str(result[0] - average[j])
			j+=1
	
#		for j in range(len(average)):
#			print predictedResult[j] - avg[j]

		print '[' + str(max(difference)) + ' ' + str(min(difference)) + ']'


def cropped_msg_cb(data):
	print 'In cropped msg cb'
	#time.sleep(100)

	#n_neighbors = int(sys.argv[1])
	n_neighbors = 20
#	prefixInput = "../learn/LocalizationTrain/15.train"
	THRESOLD = -999

	features,labels = loadData(prefixInput)
	weights = 'distance'

	print "{1} {0}-NN".format(n_neighbors,prefixInput)

	clf = neighbors.KNeighborsClassifier(n_neighbors, weights=weights)
	clf.fit(features, labels) 
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
			

	#img = cv2.imread(data.filePath)
	#print type(img)
	#img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#surfDetector = cv2.FeatureDetector_create("SURF")
	#surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")
	#keypoints = surfDetector.detect(img)
	#(keypoints, descriptors) = surfDescriptorExtractor.compute(img,keypoints)
	#print len(descriptors[0])

	imgFeature = testFeature
	result = [0.0 for i in probLabel]
	#for i in len(keypoints):
		#imgFeature.append(descriptors[i])

	objectClass =[0 for i in probLabel]

	for aFeature in imgFeature:
		#print "Object :" + str(label_list[0]),
	#for i in aFeature:
		#send aFeature as params
		aRes = NaiveFunction2(clf,result,aFeature)
		for x in range(len(aRes)):
			result[x] += aRes[x]
		#objectClass[index(min(result))] += 1


	labelResult = ''
	firstMin = min(result);
	firstLabelObject = result.index(min(result)) + 1

	result[result.index(min(result))] = 999
	secondMin = min(result);
	secondLabelObject = result.index(min(result)) + 1
	
	#for i in result:
	#	print i

	if(abs(firstMin - secondMin) < THRESOLD):
		labelResult = -1
	else:
		labelResult = firstLabelObject

	print str(labelResult) + " " + str(data.vector.x) + " " + str(data.vector.y)

	publisher.publish(str(labelResult) + " " + str(data.vector.x) + " " + str(data.vector.y))

#if firstMin - secondMin > THRESOLD:
	#print firstLabelObject
#else:
	#print secondLabelObject

def main():
	filePath_sub = rospy.Subscriber("cropped_msg",cropped_msg,cropped_msg_cb)
	#publisher = rospy.Publisher('verified_result', String)

	test_sub = rospy.Subscriber("test_input",String,test_input_cb)

	#publisher = rospy.Publisher('verified_result', String)
	rospy.init_node('Naivebayes', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
	main()
