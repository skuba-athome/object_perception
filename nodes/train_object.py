#!/usr/bin/python

import roslib
import rospy
import cv2
import os
import numpy

roslib.load_manifest('object_perception')

surf = cv2.SURF(400)
feature_directory = "/run/shm/feature"
object_feature = {}

def list_image_in_directory(dir):
    pic_dic = {}
    for object_dir in os.listdir(dir):
        object_dir_path = os.path.join(dir,object_dir)
        if not os.path.isdir(object_dir_path):
            continue
        pic_dic[str(object_dir)] = []
        for object_pic in os.listdir(object_dir_path):
            if object_pic.endswith(".jpg"):
                pic_dic[str(object_dir)].append(str(os.path.join(object_dir_path,object_pic)))
    return pic_dic

def write_feature_file(file_name,descriptors):
    file_ptr = open(file_name,'w')
    for descriptor in descriptors:
        line = map((lambda x: '%.15f' % (x)),descriptor)
        line = reduce((lambda x, y: '%s %s' % (x,y)), line)
        file_ptr.write(line+'\n')

    file_ptr.close()
    print 'Write '+file_ptr.name+' complete'

def write_kmeans_config(centers):
    file_ptr = open("centroids.config","w")
    file_ptr.write(str(len(centers))+"\n")
    for centroid_vector in centers:
        line = map((lambda x: '%.15f' % (x)),centroid_vector)
        line = reduce((lambda x, y: '%s %s' % (x,y)), line)
        file_ptr.write(line+'\n')
    file_ptr.close()
    print 'Write K-mean result file.'

def extract_feature_from_images(image_list,object_name):
    global object_feature
    feature_temporary_dir = os.path.join(feature_directory,object_name)
    if not os.path.exists(feature_temporary_dir):
        os.makedirs(feature_temporary_dir)

    object_feature[object_name] = []
    for image_name in image_list:
        image = cv2.imread(image_name, 0)
        key_points, descriptors = surf.detectAndCompute(image, None)
        object_feature[object_name].append(descriptors)
        print "Extract feature for " + image_name

        basename = os.path.splitext(os.path.basename(image_name))[0]
        write_feature_file(os.path.join(feature_temporary_dir,basename+".feature"), descriptors)

def create_train_data():
    data = []
    for object_name in object_feature:
        for descriptors in object_feature[object_name]:
            for descriptor in descriptors:
                data.append(descriptor)
    return numpy.float32(data)

def get_euclidean_distance(x,y):
    return sum(map((lambda x, y: (x-y)**2), x, y))

def get_compactness_for_groups(data,labels,centers):
    compactness_groups = {}
    distances = map((lambda x, y: get_euclidean_distance(x,centers[y[0]])), data, labels)
    for index in range(len(labels)):
        if not labels[index][0] in compactness_groups:
            compactness_groups[labels[index][0]] = 0.0
        compactness_groups[labels[index][0]] += distances[index]
    return compactness_groups

if __name__ == '__main__':
    rospy.init_node('train_object')
    object_root_dir = rospy.get_param('~object_directory',roslib.packages.get_pkg_dir('object_perception') + '/learn')
    object_dic = list_image_in_directory(object_root_dir)

    # extract SURF feature
    for object_name in object_dic:
        extract_feature_from_images(object_dic[object_name],object_name)

    # perform K-means
    kmean_max_iter = rospy.get_param('~kmean_max_iter','300')
    kmean_epsilon = rospy.get_param('~kmean_epsilon','0.00001')
    kmean_k_cluster = rospy.get_param('~kmean_k_cluster','400')
    kmean_attempts = rospy.get_param('~kmean_attempts','20')

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, int(kmean_max_iter), float(kmean_epsilon))
    train_data = create_train_data()
    compactness,labels,centers = cv2.kmeans(train_data,int(kmean_k_cluster),criteria,int(kmean_attempts),cv2.KMEANS_RANDOM_CENTERS)

    #compactness_groups = get_compactness_for_groups(train_data, labels, centers)
    #for compactness in compactness_groups:
    #    print "Compactness for ",compactness," = ",compactness_groups[compactness]
    write_kmeans_config(centers)
