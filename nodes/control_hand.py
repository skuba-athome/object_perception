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
global check
check =0 
global pub
def callback(data):
	global check 
	check = 1
	send_x = int(data.x * 100) - 5	
	send_y = int(data.y * 100 * 1.1 ) + 55
	send_z = int(data.z * 100)
#	send_x = int(math.sqrt(send_x**2 + send_y**2))
	print se.write("\x01\x7F%c"%send_x+"%c"%send_y+"%c"%send_z+"\x0F")
	rospy.loginfo(rospy.get_name()+"I heard " + str(send_x) + " " + str(send_y) + " " + str(send_z) )

def callback2(msg):
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
	elif(msg.data == "swipe"):
		send_x = 8
		send_y = 8
		send_z = 8 
	elif(msg.data == "off"):
		send_x = 7
		send_y = 7
		send_z = 7
	elif(msg.data == "water"):
		send_x = 17
		send_y = 17
		send_z = 17
	elif("release" in msg.data):
		temp = msg.data.split(":")
		send_x = 6
		send_y = 6
		send_z = int(temp[1])*100
	print se.write("\x01\x7F%c"%send_x+"%c"%send_y+"%c"%send_z+"\x0F")

def read():
	global pub
	while True:
		try:
			tmp = se.read()
			if(tmp == '\x01'):
				pub.publish('Success')
			else :
				pub.publish('Active')
		except:
			pass
		

def listener():
	global pub
	rospy.init_node('control_hand', anonymous=True)
	rospy.Subscriber("object_point", Vector3, callback)
	rospy.Subscriber("hand_cmd", String, callback2)
	pub = rospy.Publisher("arm_to_voice",String)
	#initial manipulation
	init_mani = 0
	#se.write("\x01\x7F%c"%(71)+"%c"%(82)+"\x28\x0F")	
	se.write("\x01\x7F%c"%(0)+"%c"%(0)+"\x10\x0F")
	recieve = threading.Thread(target = read)
	recieve.setDaemon(True)
	recieve.start()
	rospy.spin()

if __name__ == '__main__':
	listener()

