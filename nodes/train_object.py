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
object_histogram = {}

def list_image_in_directory(dir):
    pic_dic = {}
    for object_dir in os.listdir(dir):
        object_dir_path = os.path.join(dir, object_dir)
        if not os.path.isdir(object_dir_path):
            continue
        pic_dic[str(object_dir)] = []
        for object_pic in os.listdir(object_dir_path):
            if object_pic.endswith(".jpg"):
                pic_dic[str(object_dir)].append(str(os.path.join(object_dir_path, object_pic)))
    return pic_dic

def write_feature_file(file_name, descriptors):
    file_ptr = open(file_name, 'w')
    for descriptor in descriptors:
        line = map((lambda x: '%.15f' % (x)), descriptor)
        line = reduce((lambda x, y: '%s %s' % (x,y)), line)
        file_ptr.write(line+'\n')

    file_ptr.close()
    print 'Write '+file_ptr.name+' complete'

def write_kmeans_config(centers):
    file_ptr = open("cluster.config", "w")
    file_ptr.write(str(len(centers))+"\n")
    for centroid_vector in centers:
        line = map((lambda x: '%.15f' % (x)), centroid_vector)
        line = reduce((lambda x, y: '%s %s' % (x,y)), line)
        file_ptr.write(line+'\n')
    file_ptr.close()
    print 'Write K-mean result file.'

def extract_feature_from_images(image_list, object_name):
    global object_feature
    feature_temporary_dir = os.path.join(feature_directory, object_name)
    if not os.path.exists(feature_temporary_dir):
        os.makedirs(feature_temporary_dir)

    object_feature[object_name] = []
    for image_name in image_list:
        image = cv2.imread(image_name, 0)
        key_points, descriptors = surf.detectAndCompute(image, None)
        object_feature[object_name].append(descriptors)
        print "Extract feature for " + image_name

        basename = os.path.splitext(os.path.basename(image_name))[0]
        write_feature_file(os.path.join(feature_temporary_dir, basename+".feature"), descriptors)

def create_kmean_train_data():
    data = []
    for object_name in object_feature:
        for descriptors in object_feature[object_name]:
            for descriptor in descriptors:
                data.append(descriptor)
    return numpy.float32(data)

def create_histogram_from_features(features, knn_model):
    histogram = [0 for index in range(kmean_k_cluster)]
    for feature in features:
        retval, results, neigh_resp, dists = knn_model.find_nearest(numpy.float32([feature]), 1)
        histogram[int(retval)-1] += 1
    return histogram

def create_histogram_train_data(knn_model):
    global object_histogram
    object_histogram = {}
    for object_name in object_feature:
        object_histogram[object_name] = []
        for descriptors in object_feature[object_name]:
            object_histogram[object_name].append(create_histogram_from_features(descriptors, knn_model))

def get_svm_train_data(select_object_name):
    data = []
    labels = []
    # get object data
    for object_name in object_histogram:
        for histogram in object_histogram[object_name]:
            data.append(histogram)
            if object_name == select_object_name:
                labels.append(1)
            else:
                labels.append(-1)
    return numpy.float32(data), numpy.float32(labels)

def write_svm_config(object_name,svm_model):
    if not os.path.exists('svm_configs/'):
        os.makedirs('svm_configs/')
    file_name =  'svm_configs/'+object_name+'.dat'
    svm_model.save(file_name)
    print 'Write SVM model file(', file_name, ').'

if __name__ == '__main__':
    global kmean_k_cluster
    rospy.init_node('train_object')
    object_root_dir = rospy.get_param('~object_directory', roslib.packages.get_pkg_dir('object_perception') + '/learn')
    object_dic = list_image_in_directory(object_root_dir)

    print sum([len(object_dic[object_name]) for object_name in object_dic]),"pictures found."
    # extract SURF feature
    for object_name in object_dic:
        extract_feature_from_images(object_dic[object_name], object_name)

    # perform K-means
    kmean_max_iter = int(rospy.get_param('~kmean_max_iter', '300'))
    kmean_epsilon = float(rospy.get_param('~kmean_epsilon', '0.00001'))
    kmean_k_cluster = int(rospy.get_param('~kmean_k_cluster', '300'))
    kmean_attempts = int(rospy.get_param('~kmean_attempts', '20'))

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, kmean_max_iter, kmean_epsilon)
    train_data = create_kmean_train_data()
    compactness, labels, centers = cv2.kmeans(train_data, kmean_k_cluster, criteria, kmean_attempts, cv2.KMEANS_RANDOM_CENTERS)

    write_kmeans_config(centers)

    # create KNN
    knn_model = cv2.KNearest()
    labels = numpy.float32([index+1 for index in range(kmean_k_cluster)])
    knn_model.train(centers, labels)

    # perform SVM
    svm_c = int(rospy.get_param('~svm_c', '2'))
    svm_gamma = float(rospy.get_param('~svm_gamma', '0.05'))

    params = dict(kernel_type = cv2.SVM_RBF, svm_type = cv2.SVM_C_SVC, C = svm_c, gamma = svm_gamma)

    # create histogram train data
    create_histogram_train_data(knn_model)
    for object_name in object_histogram:
        train_data, labels = get_svm_train_data(object_name)

        # build SVM model
        svm_model = cv2.SVM()
        svm_model.train(train_data, labels, params = params)

        #test model
        predict_labels = svm_model.predict_all(train_data)
        fault = len(labels) - sum(map((lambda x, y: (x*y)), labels, predict_labels))
        print object_name, "fault : ", fault

        write_svm_config(object_name, svm_model)
