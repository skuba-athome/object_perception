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
se.timeout =  5
#se.port = sys.argv[1]
se.port = '/dev/ttyACM1'
se.open()
#======================
#print " \x01 \x02 \x03 \x04"
print se
tmp = ''
HandPublisher = rospy.Publisher('hand_state',String)
def callback(msg):
	if(msg.data == "send"):
		send_x = 15
		send_y = 15
		send_z = 15
	elif(msg.data == "back"):
		send_x = 13
		send_y = 13
		send_z = 13
	elif(msg.data == "up"):
		send_x = 0
		send_y = 0
		send_z = 20
	elif(msg.data == "initial"):
		send_x = 10
		send_y = 10
		send_z = 10	
	else :
		try:
			n = int(msg.data)
			send_x = 0
			send_y = 0 
			send_z = n
		except :
			return 0 
	print se.write("\x01\x7F%c"%send_x+"%c"%send_y+"%c"%send_z+"\x0F")
	rospy.loginfo(rospy.get_name()+"I heard " + str(send_x) + " " + str(send_y) + " " + str(send_z) )

def read():
	while True:
		tmp = se.read()
		if(tmp == '\x01'):
			#print 'ready !'
			HandPublisher.publish('1')
		else :
			#print 'not ready !'
			HandPublisher.publish('0')

def listener():
	rospy.init_node('control_hand', anonymous=True)
	rospy.Subscriber("hand_cmd", String, callback)
	#initial manipulation
	init_mani = 0
	se.write("\x01\x7F%c"%init_mani+"%c"%init_mani+"\x10\x0F")
	recieve = threading.Thread(target = read)
	recieve.setDaemon(True)
	recieve.start()
	rospy.spin()

if __name__ == '__main__':
	listener()

