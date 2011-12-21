#!/usr/bin/env python
import roslib; roslib.load_manifest('objects')
import rospy
import serial 
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys
import time
import threading
#initial==============
se = serial.Serial()
se.baudrate = 9600
se.bytesize = 8
se.timeout = 1
se.port = sys.argv[1]
se.open()
#======================
#print " \x01 \x02 \x03 \x04"
print se
def callback(data):
	send_x = int(data.z * 100) 	
	send_y = int(data.x * 100) + 55
	send_z = int(data.y * 100) + 5
	se.write("\x01\x7F%c"%send_x+"%c"%send_y+"%c"%send_z+"\x0F")
	rospy.loginfo(rospy.get_name()+"I heard " + str(send_x) + " " + str(send_y) + " " + str(send_z) )

def read():
	while True:
		tmp = se.read()
		print tmp

def listener():
	rospy.init_node('control_hand', anonymous=True)
	rospy.Subscriber("object_point", Vector3, callback)
	#initial manipulation
	init_mani = 0
#	se.write("\x01\x7F%c"%init_mani+"%c"%init_mani+"\x14\x0F")
#	recieve = threading.Thread(target = read)
#	recieve.setDaemon(True)
#	recieve.start()
	rospy.spin()

if __name__ == '__main__':
	listener()

