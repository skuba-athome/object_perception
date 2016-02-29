#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

class Capture:
    def __init__(self):
        rospy.Subscriber("/external_cam/image_color", Image, self.update_image, queue_size=1)
        self.bridge = CvBridge()
        self.history_location = os.path.join(rospkg.RosPack().get_path('object_recognition_v2'),
                        'tester')

    def update_image(self, data):
        self.image = data
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imwrite(self.history_location, frame)

if __name__ == '__main__':
    Capture()
    rospy.spin()
