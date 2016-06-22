#!/usr/bin/env python
import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from object_recognition_v2.srv import AddXYZ, AddXYZResponse
from object_recognition_v2.msg import ObjectRecognition, ObjectRecognitions
import tf
import numpy as np

class XYZPoint:

    def __init__(self):
        self.point_cloud2 = None
        self.tf_listener = None
        self.header = None
        self.point_cloud2_subscriber = None
        self.init_server()

    def handle_point_cloud2(self, data):
        rospy.loginfo("XYZ service: Start Call service.")
        rospy.loginfo("XYZ service: Start Subscribe PointCloud2.")
        self.point_cloud2_subscriber = rospy.Subscriber("/depth_registered/depth_registered/points", PointCloud2, self.update_point_cloud2, queue_size=1)
        objects = data.objects
        while self.point_cloud2 == None:
            rospy.loginfo("XYZ service: Wait for income PointCloud2.")
            rospy.sleep(1)
        point_cloud2_temp = self.point_cloud2

        for thing in objects:
            points = []
            centriods = []
            points.append([int(thing.point.x), int(thing.point.y)])
            points.append([int(thing.point.x+1), int(thing.point.y)])
            points.append([int(thing.point.x), int(thing.point.y-1)])
            points.append([int(thing.point.x), int(thing.point.y+1)])
            points.append([int(thing.point.x-2), int(thing.point.y)])
            points.append([int(thing.point.x+2), int(thing.point.y)])
            points.append([int(thing.point.x-1), int(thing.point.y)])
            points.append([int(thing.point.x), int(thing.point.y-2)])
            points.append([int(thing.point.x), int(thing.point.y+2)])
            centriods = pc2.read_points(point_cloud2_temp, skip_nans=False, field_names=("x","y","z"), uvs=points)
            for centriod in centriods:
                if not (np.isnan(centriod[0]) or np.isnan(centriod[1]) or np.isnan(centriod[2])):
                    self.header.stamp = rospy.Time(0)
                    point_tf = self.tf_listener.transformPoint('base_link', PointStamped(header=self.header, point=Point(x=centriod[0], y=centriod[1], z=centriod[2])))
                    thing.centriod.x = point_tf.point.x
                    thing.centriod.y = point_tf.point.y
                    thing.centriod.z = point_tf.point.z
                    thing.header =  point_tf.header
                    break
            if thing.centriod.x == 0 and thing.centriod.y == 0 and thing.centriod.z == 0:
                thing.centriod.x = np.nan
                thing.centriod.y = np.nan
                thing.centriod.z = np.nan
            rospy.loginfo("XYZ service: Name: {0} Found At: X:{1:<4} Y:{2:<4} Z:{3:<4}".format(thing.name,
                            thing.centriod.x, thing.centriod.y, thing.centriod.z))
        rospy.loginfo("XYZ service: Send Data back")
        return AddXYZResponse(objects)

    def update_point_cloud2(self, data):
        self.point_cloud2 = data
        self.point_cloud2_subscriber.unregister()
        rospy.loginfo("XYZ service: Start Subscribe PointCloud2.")


    def init_server(self):
        rospy.init_node('get_xyz_from_uv_server', anonymous=False)
        self.tf_listener = tf.TransformListener()

        self.header = Header()
        self.header.frame_id = "external_cam"
        s = rospy.Service('get_xyz_from_uv', AddXYZ, self.handle_point_cloud2)

        rospy.loginfo("XYZ service: init service")

if __name__ == '__main__':
    XYZPoint()
    rospy.spin()
