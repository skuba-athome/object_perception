#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
import rospkg
import os
from cv_bridge import CvBridge, CvBridgeError
from object_recognition_v2.srv import AddXYZ
from object_recognition_v2.msg import ObjectRecognition, ObjectRecognitions
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import String
from datetime import datetime

class Size:

    def __init__(self, width, height):
        self.width = width;
        self.height = height;

class HOGReconition:

    def __init__(self):
        self.bridge = None
        self.hogs = None
        self.add_xyz = None
        self.init_node()

    def load_ymls(self, location):
        yml = open(location, "r")
        hogs = []
        sizes = {}
        for line in yml.readlines():
            data = line.strip().split("\t")
            if data[1] == "ON":
                size = data[0].split("--")[1].split("x")
                width = size[0]
                height = size[1]
                sizes[data[0]] = Size(width = width, height = height)
                hog = cv2.HOGDescriptor()
                hogs.append((data[0], hog))
                print "Loaded: {0:<70}".format(data[0]),
                print hog.load(data[0])
        return hogs

    def detect_object(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        things = ObjectRecognitions()

        thing = ObjectRecognition(name=String("TEST"), point=Pose2D(x=640, y=360))
        # thing = ObjectRecognition(name=String("TEST"), point=Pose2D(x=320, y=240))
        things.objects.append(thing)
        cv2.rectangle(frame, (639, 359),
                            (641, 361),
                            (0,0,255), 3)
        # for name, hog in self.hogs:
        #     point = hog.detectMultiScale(frame)
        #     if point != ((), ()):
        #         print name, point
        #         center = (float(round(point[0][0][0]+1.0*point[0][0][2]/2)),
        #                     float(round(point[0][0][1]+1.0*point[0][0][3]/2)))
        #         image = frame[point[0][0][1]:point[0][0][1]+point[0][0][3], point[0][0][0]:point[0][0][0]+point[0][0][2]]
        #         thing = ObjectRecognition(name=String(name), point=Pose2D(x=center[0], y=center[1]))
        #
        #         things.objects.append(thing)
        #         print "center: x: {1:>4}, y:{1:>4}".format(center[0], center[1])
        #         cv2.rectangle(frame, (point[0][0][0], point[0][0][1]),
        #                         (point[0][0][0]+point[0][0][2], point[0][0][1]+point[0][0][3]),
        #                         (0,0,255), 3)
        #         cv2.putText(frame, name, (point[0][0][0], point[0][0][1]),
        #                         cv2.FONT_HERSHEY_PLAIN, 1.3, (47, 211, 25),2)
        #         cv2.imshow('object_view', image)
        if things.objects != []:
            try:
                things = self.add_xyz(things.objects)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        # rospy.Publisher("/object_recognition", ObjectRecognitions, things, queue_size=1)
        # cv2.putText(frame, ""+str(things.objects[0].centriod), (630, 350),
        #                     cv2.FONT_HERSHEY_PLAIN, 1.3, (47, 211, 25),2)
        cv2.imshow('recognize_object_view', frame)
        cv2.waitKey(30)

    def create_dir(self):
        history_location = os.path.join(rospkg.RosPack().get_path('object_recognition_v2'),
                        datetime.today().isoformat(" "))
        os.makedirs(history_location)

    def load_ymls_from_dir(self):
        yml_location = os.path.join(rospkg.RosPack().get_path('object_recognition_v2'),
                        "classifiers/list_yml.txt")
        self.hogs = self.load_ymls(yml_location)

    def connect_service(self):
        rospy.wait_for_service('get_xyz_from_uv')
        self.add_xyz = rospy.ServiceProxy('get_xyz_from_uv', AddXYZ)

    def init_tool(self):
        self.bridge = CvBridge()

        cv2.namedWindow("recognize_object_view", 1)
        cv2.namedWindow("object_view", 1)

    def init_node(self):
        rospy.init_node('object_recognition_node',anonymous=False)
        self.create_dir()
        self.load_ymls_from_dir()
        self.connect_service()
        self.init_tool()

        rospy.Subscriber("/external_cam/image_color", Image, self.detect_object, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    HOGReconition()
