print(__doc__)

import numpy as np

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler

import os
import cv2
import numpy

##############################################################################
# Generate sample data
surf = cv2.SURF(400)
feature_directory = "/run/shm/feature"
#object_name = "pringles/"
#directory = "/home/skuba/skuba_athome/object_perception/data/"
directory = "/home/skuba/skuba_athome/object_perception/object_recognition/learn/PicCut/"
centers =[]
for folder_name in os.listdir(directory):#+object_name):
#    for file_name in os.listdir(directory+folder_name+'/train/'):
#        print directory+folder_name+'/train/'+file_name
#        if file_name.endswith(".jpg") or file_name.endswith(".png"):
#            print directory+folder_name+'/train/'+file_name
#            #print  directory+folder_name+file_name
#            image = cv2.imread(directory+folder_name+'/train/'+file_name, 0)

    for file_name in os.listdir(directory+folder_name):
        if file_name.endswith(".jpg") or file_name.endswith(".png"):
            print directory+folder_name+file_name
            #print  directory+folder_name+file_name
            print directory+folder_name+'/'+file_name
            image = cv2.imread(directory+folder_name+'/'+file_name, 0)
            #image = cv2.imread(directory+object_name+file_name, 0)
            key_points, descriptors = surf.detectAndCompute(image, None)
            #centers.append(descriptors)
            if len(key_points) == 0:
                continue
            for feature in descriptors:
                centers.append(feature)

print len(centers),' ',len(centers[0])
#centers = [[1, 1,], [-1, -1], [1, -1]]
X = np.asarray(centers)

#X, labels_true = make_blobs(n_samples=750, centers=centers, cluster_std=0.4,random_state=0)
#print type(centers), ' ' , type(X)
#print type(X)
#X = StandardScaler().fit_transform(X)

##############################################################################
# Compute DBSCAN
db = DBSCAN(eps=0.3, min_samples=10).fit(X)

core_samples = db.core_sample_indices_
labels = db.labels_

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

print('Estimated number of clusters: %d' % n_clusters_)
#print("Homogeneity: %0.3f" % metrics.homogeneity_score(labels_true, labels))
#print("Completeness: %0.3f" % metrics.completeness_score(labels_true, labels))
#print("V-measure: %0.3f" % metrics.v_measure_score(labels_true, labels))
#print("Adjusted Rand Index: %0.3f"
      #% metrics.adjusted_rand_score(labels_true, labels))
#print("Adjusted Mutual Information: %0.3f"
      #% metrics.adjusted_mutual_info_score(labels_true, labels))
#print("Silhouette Coefficient: %0.3f"
#      % metrics.silhouette_score(X, labels))

##############################################################################
# Plot result
import pylab as pl

# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = pl.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = 'k'
        markersize = 6
    class_members = [index[0] for index in np.argwhere(labels == k)]
    cluster_core_samples = [index for index in core_samples
                            if labels[index] == k]
    for index in class_members:
        x = X[index]
        if index in core_samples and k != -1:
            markersize = 14
        else:
            markersize = 6
        pl.plot(x[0], x[1], 'o', markerfacecolor=col,
                markeredgecolor='k', markersize=markersize)

pl.title('Estimated number of clusters: %d' % n_clusters_)
pl.show()

