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
class run:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.tmpx= 0
		self.tmpy= 0
		self.pub = rospy.Publisher('mv_state',String)
	def fwd(self,s): #forward
		se.write("\x01\x7F\x08\x00%c\x00\x0F"%s)

	def bwd(self,s): #backward
		se.write("\x01\x7F\x0A\x00%c\x00\x0F"%s)
	def sl(self,s): #shift left
		#print "shift left %d"%s
		se.write("\x01\x7F\x0C%c\x00\x00\x0F"%s)
	def sr(self,s): #shift right
		#print "shift right %d"%s
		se.write("\x01\x7F\x08%c\x00\x00\x0F"%s)

	def tl(self,s): #turn left 
		#print "shift left %d"%s
		se.write("\x01\x7F\x08\x00\x00%c\x0F"%s)

	def tr(self,s): #turn right
		#print "shift right %d"%s
		se.write("\x01\x7F\x09\x00\x00%c\x0F"%s)
	
	def callback(self,data):
		self.x = 0
		self.y = 0
		print data.x , data.z , self.y
		rate = 1
		if ( data.x < 0 ):
			while( self.x > data.x*rate ) :
				if( self.x - self.tmpx < 0.15 ):
					self.x = self.tmpx 
				self.sl(5)
				print "x" , self.x , data.x
				time.sleep(0.05)
		elif (data.x > 0 ): 
			while ( self.x < data.x*rate ) :
				if (  self.tmpx - self.x < 0.15 ):
					self.x = self.tmpx
				self.sr(5)
				print "x",self.x , data.x
				time.sleep(0.05)
		while ( float(self.y) < float(data.z - 0.65)) :
			if( self.tmpy - self.y < 0.15 ):
				self.y = self.tmpy
			self.fwd(5)
			print "y",self.y , data.z
			time.sleep(0.05)
		
		self.pub.publish('finish')	
		se.write("\x01\x7F\x00\x00\x00\x00\x0F")

	def read(self):
		while( True ):
			serial_str = []
			for i in range(13):
				serial_str.append(se.read())
			if(len(serial_str) == 13):
				if(serial_str[0]=='\x01'):
					ctrl = serial_str[2] 
					Sx = -1 if (int( ctrl.encode('hex') ,16) & 32) / 32 else 1
					Sy = -1 if (int(ctrl.encode('hex'),16)&16) / 16 else 1
					Sth = -1 if (int(ctrl.encode('hex'),16)&8) / 8 else 1
					SVx = -1 if (int(ctrl.encode('hex'),16)&4) / 4 else 1
					SVy = -1 if (int(ctrl.encode('hex'),16)&2) / 2 else 1
					SVth = -1 if (int(ctrl.encode('hex'),16)&1)  else 1
					self.tmpx =  Sx*int((serial_str[3]+serial_str[4]).encode('hex'),16)/100.0
					self.tmpy = Sy*int( (serial_str[5]+serial_str[6]).encode('hex') ,16)/100.0
					th = Sth*int(serial_str[7].encode('hex'),16)/10.0
					vx =  SVx*int(serial_str[8].encode('hex') ,16)/500.0
					vy =  SVy*int(serial_str[9].encode('hex') ,16)/500.0
					vth = SVth*int(serial_str[10].encode('hex') ,16)/5.0
					#print self.x,self.y 
	def listener(self):
		rospy.init_node('serial_skuba', anonymous=True)
		rospy.Subscriber("object_point2",Vector3, self.callback)
		se.write("\x01\x7F\x00\x00\x00\x00\x0F")
		recieve = threading.Thread(target = self.read)
		recieve.setDaemon(True)
		recieve.start()
		rospy.spin()

if __name__ == '__main__':
	tmp = run()
	tmp.listener()

