#! /usr/bin/env python
import rospy
import roslib
import actionlib
from clothing_type_classification.msg import FindClothesGoal, FindClothesAction


if __name__ == '__main__':
    rospy.init_node('find_clothes_client')
    client = actionlib.SimpleActionClient('clothes_detection_dummy', FindClothesAction)
    client.wait_for_server()

    goal = FindClothesGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    result = client.get_result()
    print "Result from Server : " + str(result)