#!/usr/bin/env python
import rospy
from object_recognition_msgs.msg import TableArray
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
import np

class GuessObject:

    def __init__(self):
        rospy.init_node("guess_detection")
        # self.nearest_table = None
        # rospy.Subscriber("ork/table_array", TableArray, self.receive_table_array)
        rospy.Subscriber("ork/tabletop/clusters", MarkerArray, self.receive_clusters)
        self.object = rospy.Publisher("~cluster", Marker, queue_size=10)
        self.object_name = rospy.Publisher("~guess_object", String, queue_size=1)
        rospy.spin()

    def poly_area_2D(self, pts):
        lines = np.hstack([pts, np.roll(pts, -1, axis=0)])
        area = 0.5 * abs(sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in lines))
        return area

    # def receive_clusters(self, clusters):
        # if not self.nearest_table:
        #     return
        # z_plane = self.nearest_table.pose.position.z
        # nearest = None
        # nearest_z  = 99
        # width = 0
        # height = 0
        # depth = 0

        # for cluster in clusters.markers:
        #     size = len(cluster.points)

        #     if size < 300:
        #         return

        #     z = 0
        #     _min_x = 99
        #     _min_y = 99
        #     _min_z = 99

        #     _max_x = -99
        #     _max_y = -99
        #     _max_z = -99
        #     for point in cluster.points:
        #         z += point.z
        #         _min_x = min(_min_x, point.x)
        #         _min_y = min(_min_y, point.y)
        #         _min_z = min(_min_z, point.z)

        #         _max_x = max(_max_x, point.x)
        #         _max_y = max(_max_y, point.y)
        #         _max_z = max(_max_z, point.z)


        #     z = z/size
        #     _width = abs(_min_x - _max_x)
        #     _height = abs(_min_y - _max_y)
        #     _depth = abs(_min_z - _max_z)

        #     if _width < 0.1 or _height < 0.1:
        #         return

        #     # print nearest_z, z
        #     if nearest_z > z:
        #         nearest_z = z
        #         nearest = cluster
        #         width = _width
        #         height = _height
        #         depth = _depth
        #         # print '-----------'

        #     # print width, height, depth
        # self.object.publish(nearest)
        # if nearest_z < 3 and height < 0.3:
        #     print "Width: ", width, "Height: ", height, "Depth: ",  depth
        #     self.object.publish(nearest)
        #     self.object_name.publish(String("pet"))

        # elif nearest_z < 3 and 0.8 < height < 1.6:
        #     print "Width: ", width, "Height: ", height, "Depth: ", depth
        #     self.object.publish(nearest)
        #     self.object_name.publish(String("obstacle"))

            # elif z < 3 and width < 0.6 and height < 1.6:
            #     print "Width: ", width, "Height: ", height, "Depth: ", depth
            #     self.object.publish(cluster)
            #     self.object_name.publish(String("people"))

    # def receive_table_array(self, table_array):
    #     min_z = 99
        # self.nearest_table = None
        # for table in table_array.tables:

            # position = table.pose.position
            # orientation = table.pose.orientation
            # convex_hull = table.convex_hull


            # if position.z > 1.5:
            #     continue

            # if min_z > position.z:
            #     self.nearest_table = table
            #     min_z = position.z

            # if position.x > 1.5:
            #     continue
            #
            # if position.y > 1.5:
            #     continue
            # pts = [[i.x, i.y] for i in convex_hull]
            # area = self.poly_area_2D(pts)
            # print area
            # if area < 200:
            #     pass
        # print self.nearest_table.pose

if __name__ == "__main__":
    GuessObject()
