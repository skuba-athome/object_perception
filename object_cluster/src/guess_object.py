#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from object_cluster.srv._Guess import GuessResponse
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Empty, String
from geometry_msgs.msg import PointStamped, Point
from object_cluster.srv import Guess

import tf

class GuessObject:

    def __init__(self):
        self.point_cloud2 = None
        self.tf_listener = None
        self.header = None
        self.point_cloud2_subscriber = None
        self.init_server()

    def handle_point_cloud2(self, data):
        rospy.loginfo("XYZ service: Start Call service.")
        rospy.loginfo("XYZ service: Start Subscribe PointCloud2.")
        self.point_cloud2_subscriber = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.update_point_cloud2, queue_size=1)
        while self.point_cloud2 == None:
            rospy.loginfo("XYZ service: Wait for income PointCloud2.")
            rospy.sleep(1)
        point_cloud2_temp = self.point_cloud2

        position = pc2.read_points(point_cloud2_temp, skip_nans=False, field_names=("x","y","z"))
        y = -99
        for point in position:
            if abs(point[0]) < 1 and point[2] < 3:
                y = max(y, point[1])

        rospy.loginfo("XYZ service: Send Data back")
        print y, '------------'
        point_tf = self.tf_listener.transformPoint('base_link', PointStamped(header=self.header, point=Point(x=0, y=y, z=0)))
        if point_tf.point.z  < 0.4:
            return GuessResponse(guess=String("pet"))
        else:
            return GuessResponse(guess=String("obstacle"))


    def update_point_cloud2(self, data):
        self.point_cloud2 = data
        self.point_cloud2_subscriber.unregister()
        rospy.loginfo("XYZ service: Start Subscribe PointCloud2.")


    def init_server(self):
        rospy.init_node('xyz', anonymous=False)
        self.tf_listener = tf.TransformListener()

        self.header = Header()
        self.header.frame_id = "camera_depth_optical_frame"
        s = rospy.Service('~get', Guess, self.handle_point_cloud2)

        rospy.loginfo("XYZ service: init service")

if __name__ == '__main__':
    GuessObject()
    rospy.spin()
