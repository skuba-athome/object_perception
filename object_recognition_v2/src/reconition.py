#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
import rospkg
import os
from cv_bridge import CvBridge, CvBridgeError
from object_recognition_v2.srv import AddXYZ
from object_recognition_v2.msg import ObjectRecognition, ObjectRecognitions
from object_recognition_v2.msg import RecognizeObjectsAction
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import String
from datetime import datetime
import actionlib

class Size:

    def __init__(self, width, height):
        self.width = width;
        self.height = height;

class HOGReconition:

    def __init__(self):
        self.bridge = None
        self.hogs = None
        self.add_xyz = None
        self.image = None
        self.server = None
        self.history_location = None
        self.image_subscriber = None
        self.init_node()

    def load_ymls(self, location):
        yml = open(location, "r")
        hogs = []
        sizes = {}
        rospy.loginfo("START LOAD YML")
        for line in yml.readlines():
            data = line.strip().split("\t")
            if data[1] == "ON":
                size = data[0].split("--")[1].split("x")
                width = size[0]
                height = size[1]
                sizes[data[0]] = Size(width = width, height = height)
                hog = cv2.HOGDescriptor()
                hogs.append((data[0], hog))
                rospy.loginfo("LOADED: {0:<70} {1}".format(data[0].split('object_recognition_v2/')[1], hog.load(data[0])))
        return hogs

    def update_image(self, data):
        self.image = data
        self.image_subscriber.unregister()

    def recognize_objects(self, goal):
        self.image_subscriber = rospy.Subscriber("/external_cam/image_color", Image, self.update_image, queue_size=1)
        while self.image == None:
            rospy.loginfo("Wait for income Image.")
            rospy.sleep(1)
        frame = self.image
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        names = [i.data for i in goal.names]
        things = ObjectRecognitions()
        rospy.loginfo("Input: {0}".format(names))

        # thing = ObjectRecognition(name=String("TEST"), point=Pose2D(x=640, y=360))
        # things.objects.append(thing)
        # cv2.rectangle(frame, (639, 359),
        #                     (641, 361),
        #                     (0,0,255), 3)
        # self.server.publish_feedback("start scan")

        for name, hog in self.hogs:
            if name not in names and names[0] != '*':
                continue
            rospy.loginfo("Check for: {0}".format(name))
            name = name.split("--")[0].split("/")[-1]
            points = hog.detectMultiScale(frame)
            num = len(points[1])
            for i in range(num):
                point = points[0][i]
                confident = points[1][i]
                if confident > 0:
                    center = (float(round(point[0]+1.0*point[2]/2)),
                                float(round(point[1]+1.0*point[3]/2)))
                    image = frame[point[1]:point[1]+point[3], point[0]:point[0]+point[2]]
                    thing = ObjectRecognition(name=String(name), point=Pose2D(x=center[0], y=center[1]))
                    things.objects.append(thing)

                    rospy.loginfo("FOUND: {0} CENTER: x: {1:>4}, y:{2:>4}".format(name, center[0], center[1]))
                    cv2.rectangle(frame, (point[0], point[1]),
                                    (point[0]+point[2], point[1]+point[3]),
                                    (0,0,255), 3)
                    cv2.putText(frame, name, (point[0], point[1]),
                                    cv2.FONT_HERSHEY_PLAIN, 1.3, (47, 211, 25),2)
        filename = os.path.join(self.history_location, datetime.today().isoformat(" ") + ".jpg")
        cv2.imwrite(os.path.join(self.history_location, filename), frame)
        rospy.loginfo("SAVED : {0}".format(filename))
        if things.objects != []:
            try:
                things = self.add_xyz(things.objects)
            except rospy.ServiceException, e:
                rospy.logerr("XYZ Service call failed: %s"%e)
        # rospy.Publisher("/object_recognition", ObjectRecognitions, things, queue_size=1)
        self.server.set_succeeded(things)
        # cv2.putText(frame, ""+str(things.objects[0].centriod), (630, 350),
        #                     cv2.FONT_HERSHEY_PLAIN, 1.3, (47, 211, 25),2)
        # cv2.imshow('recognize_object_view', frame)
        # cv2.waitKey(30)

    def create_dir(self):
        self.history_location = os.path.join(rospkg.RosPack().get_path('object_recognition_v2'),
                        'history', datetime.today().isoformat(" "))
        os.makedirs(self.history_location)

    def load_ymls_from_dir(self):
        yml_location = os.path.join(rospkg.RosPack().get_path('object_recognition_v2'),
                        "classifiers/list_yml.txt")
        self.hogs = self.load_ymls(yml_location)

    def connect_service(self):
        rospy.wait_for_service('get_xyz_from_uv')
        self.add_xyz = rospy.ServiceProxy('get_xyz_from_uv', AddXYZ)

    def init_tool(self):
        self.bridge = CvBridge()

        # cv2.namedWindow("recognize_object_view", 1)
        # cv2.namedWindow("object_view", 1)

    def init_node(self):
        rospy.init_node('object_recognition_node',anonymous=False)
        self.create_dir()
        self.load_ymls_from_dir()
        self.connect_service()
        self.init_tool()

        # rospy.Subscriber("/external_cam/image_color", Image, detect_object, queue_size=1)
        # rospy.Subscriber("/external_cam/image_color", Image, self.update_image, queue_size=1)

        self.server = actionlib.SimpleActionServer('/object/recognize_objects', RecognizeObjectsAction, self.recognize_objects, False)
        self.server.start()

if __name__ == '__main__':
    HOGReconition()
    rospy.spin()
