#!/usr/bin/env python
import roslib; roslib.load_manifest('control_skuba')
import rospy
import serial 
from std_msgs.msg import String
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
def fwd(s): #forward
	se.write("\x01\x7F\x08\x00%c\x00\x0F"%s)

def bwd(s): #backward
	se.write("\x01\x7F\x0A\x00%c\x00\x0F"%s)
def sl(s): #shift left
	print "shift left %d"%s
	se.write("\x01\x7F\x0C%c\x00\x00\x0F"%s)
def sr(s): #shift right
	print "shift right %d"%s
	se.write("\x01\x7F\x08%c\x00\x00\x0F"%s)

def tl(s): #turn left
	print "shift left %d"%s
	se.write("\x01\x7F\x08\x00\x00%c\x0F"%s)

def tr(s): #turn right
	print "shift right %d"%s
	se.write("\x01\x7F\x09\x00\x00%c\x0F"%s)

def callback(data):
	tmp = data.data.split()
	if(tmp[0]=='f') : fwd(int(tmp[1]))
	if(tmp[0]=='b') : bwd(int(tmp[1]))
	if(tmp[0]=='sl') : sl(int(tmp[1]))
	if(tmp[0]=='sr') : sr(int(tmp[1]))
	if(tmp[0]=='tr') : tr(int(tmp[1]))
	if(tmp[0]=='tl') : tl(int(tmp[1]))
	rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def read():
	while True:
		tmp = se.read()
		print tmp

def listener():
	rospy.init_node('serial_skuba', anonymous=True)
	rospy.Subscriber("control_module", String, callback)
	recieve = threading.Thread(target = read)
	recieve.setDaemon(True)
	recieve.start()
	rospy.spin()

if __name__ == '__main__':
	listener()

