import os
import cv2
import numpy
import math

def distance(x, y):
    return (x-y)**2

def diff(feature1,feature2):
    distance_list = map(distance, feature1, feature2)
    return math.sqrt(sum(distance_list))

surf = cv2.SURF(400)
feature_directory = "/run/shm/feature"
#object_name = "pringles/"
directory = "/home/skuba/skuba_athome/object_perception/learn/PicCut/"
centers =[]
#for folder_name in os.listdir(directory):#+object_name):
#    for file_name in os.listdir(directory+folder_name+'/train/'):
#        print directory+folder_name+'/train/'+file_name
#        if file_name.endswith(".jpg") or file_name.endswith(".png"):
#            print directory+folder_name+'/train/'+file_name
#            #print  directory+folder_name+file_name
#            image = cv2.imread(directory+folder_name+'/train/'+file_name, 0)
file_name = "/home/skuba/.ros/features"
filePtr = open(file_name,"r")
feature_list = []
for line in filePtr:
    attribute = line.strip().split(" ")
    feature_list.append(map(float,attribute))

print "reading feature finished"
min_value=999999
max_value = -9999999

feature_count = len(feature_list)

for i in range(feature_count):
    const_feature = [feature_list[i] for k in range(feature_count)]
    distance_vector = map(diff, const_feature, feature_list)
    if i % 10 == 0:
        print "Progress : ", (i*1.0/feature_count)*100.0
    for a_distance in distance_vector:
            tmp = a_distance
            if tmp > max_value:
                max_value = tmp
            if tmp < min_value:
                min_value = tmp

print "min_value : " + str(min_value) + " max_value : " + str(max_value)
        #if file_name.endswith(".jpg") or file_name.endswith(".png"):
            #print directory+folder_name+file_name
            #print  directory+folder_name+file_name
            #print directory+folder_name+'/'+file_name
            #image = cv2.imread(directory+folder_name+'/'+file_name, 0)
            #image = cv2.imread(directory+object_name+file_name, 0)
            #key_points, descriptors = surf.detectAndCompute(image, None)
            #centers.append(descriptors)
            #if len(key_points) == 0:
            #    continue
            #for feature in descriptors:
            #    centers.append(feature)

#f = open('/home/skuba/.ros/features','w')
#for i in range(len(centers)):
#        line = map((lambda x: '%.15f' % (x)), centers[i])
#        line = reduce((lambda x, y: '%s %s' % (x,y)), line)
#        f.write(line+'\n') 

        
#    for j in range(len(centers)):
#        if i!=j:
            #f.write(str(diff(centers[i],centers[j]))+'\n') 
    #f.close() 
