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

class Area:
    def __init__(self, x1, y1, x2, y2, confident, name):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.confident = confident
        self.name = name
    def __repr__(self):
        return "x1:{0} x2:{1} y1:{2} y2:{3} confident: {4} name:{5}".format(self.x1, self.x2, self.y1, self.y2, self.confident, self.name)


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
        self.seq = 0

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
                path = os.path.join(self.rospack.get_path('object_recognition_v2'), data[0].split('object_recognition_v2/')[1])
                rospy.loginfo("LOADED: {0:<70} {1}".format(data[0].split('object_recognition_v2/')[1], hog.load(path)))
        return hogs

    def update_image(self, data):
        self.image = data
        self.image_subscriber.unregister()

    def isOverlab(self, area1, area2):
        return self.range_overlap(area1.x1, area1.x2, area2.x1, area2.x2) and self.range_overlap(area1.x1, area1.x2, area2.x1, area2.x2)

    def range_overlap(self, a_min, a_max, b_min, b_max):
        '''Neither range is completely greater than the other
        '''
        return (a_min <= b_max) and (b_min <= a_max)

    def remove_overlap_objects(self, objects):
        temp_object = []
        for i in objects:
            temp = i
            for j in objects:
                if temp == j:
                    continue
                if self.isOverlab(temp, j) or self.isOverlab(j, temp):
                    print temp.name , temp.confident, '****', j.name, j.confident
                    if temp.confident > j.confident:
                        temp = temp
                    else:
                        temp = j
            print temp.name, temp not in temp_object
            if temp not in temp_object:
                is_vaild = True
                while True:
                    for j in temp_object:
                        if temp == j:
                            continue
                        if self.isOverlab(temp, j) or self.isOverlab(j, temp):
                            if temp.confident > j.confident:
                                print temp.name,'------', j.name
                                temp_object.pop(j)
                                temp_object.append(temp)
                                break
                            else:
                                is_vaild = False
                    break
                if is_vaild:
                    temp_object.append(temp)
        return temp_object

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
        objects = []
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
                if confident > 0.3:
                    area = Area(point[0], point[1], point[0]+point[2], point[1]+point[3], confident=confident, name=name)
                    objects.append(area)

                    # center = (float(round(point[0]+1.0*point[2]/2)),
                    #             float(round(point[1]+1.0*point[3]/2)))
                    # image = frame[point[1]:point[1]+point[3], point[0]:point[0]+point[2]]
                    # thing = ObjectRecognition(name=String(name), point=Pose2D(x=center[0], y=center[1]))
                    # things.objects.append(thing)
                    #
                    # rospy.loginfo("FOUND: {0} CENTER: x: {1:>4}, y:{2:>4}".format(name, center[0], center[1]))
                    # cv2.rectangle(frame, (point[0], point[1]),
                    #                 (point[0]+point[2], point[1]+point[3]),
                    #                 (0,0,255), 3)
                    # cv2.putText(frame, name, (point[0], point[1]),
                    #                 cv2.FONT_HERSHEY_PLAIN, 1.3, (47, 211, 25),2)

        temp_object = self.remove_overlap_objects(objects)

        for area in temp_object:
            cv2.rectangle(frame, (area.x1, area.y1),(area.x2, area.y2) ,(0,0,255), 3)
            cv2.putText(frame, area.name + " " +  str(area.confident), (area.x1, area.y1), cv2.FONT_HERSHEY_PLAIN, 1.3,(47, 211, 25),2)

        for area in temp_object:
            center = (float(round(area.x1+1.0*area.x2/2)),
                        float(round(area.y1+1.0*area.y2/2)))
            thing = ObjectRecognition(name=String(area.name), confident=area.confident, point=Pose2D(x=center[0], y=center[1]), image=frame[area.x1:area.x2, area.y1:area.y2])
            things.objects.append(thing)
        filename = os.path.join(self.history_location, datetime.today().isoformat(" ") + ".jpg")
        cv2.imwrite(os.path.join(self.history_location, filename), frame)
        rospy.loginfo("SAVED : {0}".format(filename))
        if things.objects != []:
            try:
                things = self.add_xyz(things.objects)
            except rospy.ServiceException, e:
                rospy.logerr("XYZ Service call failed: %s"%e)
        # rospy.Publisher("/object_recognition", ObjectRecognitions, things, queue_size=1)
        things.header.stamp = rospy.Time(0)
        things.header.seq = self.seq
        things.header.frame_id = "external_cam"
        self.seq += 1
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
        self.rospack = rospkg.RosPack()
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
