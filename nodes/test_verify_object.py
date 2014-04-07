#!/usr/bin/env python
import roslib
import rospy

roslib.load_manifest('object_perception')
from object_perception.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('verifyObject')

    verify_object = rospy.ServiceProxy('verifyObject', verifyObject)
    #print dir()
    response = verify_object("/run/shm/test.jpg")
    print response

