#!/usr/bin/env python
import roslib; roslib.load_manifest('objects')
import rospy
import serial 
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import sys
import time
import threading
import math
#initial==============
se = serial.Serial()
se.baudrate = 57600
se.bytesize = 8
se.timeout = 5
se.port = sys.argv[1]
se.open()
#======================
#print " \x01 \x02 \x03 \x04"
print se
tmp = ''
def callback(data):
	send_x = int(data.z * 100) 	
	send_y = int(data.x * 100 *1.1) + 60
	send_z = int(data.y * 100)
#	send_x = int(math.sqrt(send_x**2 + send_y**2))
	print se.write("\x01\x7F%c"%send_x+"%c"%send_y+"%c"%send_z+"\x0F")
	rospy.loginfo(rospy.get_name()+"I heard " + str(send_x) + " " + str(send_y) + " " + str(send_z) )

def read():
	while True:
		tmp = se.read()
#		if(tmp == '\x01'):
#			print 'ready !'
#		else :
#			print 'not ready !'

def listener():
	rospy.init_node('control_hand', anonymous=True)
	rospy.Subscriber("object_point", Vector3, callback)
	#initial manipulation
	init_mani = 15
	se.write("\x01\x7F%c"%init_mani+"%c"%init_mani+"\x0F\x0F")	
	recieve = threading.Thread(target = read)
	recieve.setDaemon(True)
	recieve.start()
	rospy.spin()

if __name__ == '__main__':
	listener()

