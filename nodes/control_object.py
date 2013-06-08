#!/usr/bin/env python
import roslib; roslib.load_manifest('objects')
import rospy
from std_msgs.msg import String

object_list = []
object_dic = {}
surf_candidate = []
color_candidate = []
#publisher
pub_filename = []
pub_main = []
pub_pointCheck = []
pub_show = []
pub_extract = []
object_index = 0
candidate_index = 0

def decision(data):
	global surf_candidate,color_candidate,pub_main,candidate_index,pub_show
	
	if(len(surf_candidate) == len(color_candidate)):
		print color_candidate
		a = float(color_candidate[0][0])
		b = float(color_candidate[1][0])
			
		if(a <= b):
			if(a < object_dic[object_list[object_index-1]][1]):
				temp,x = surf_candidate[0][1].split('=')
				temp,y = surf_candidate[0][2].split('=')
				pub_show.publish('%s %s %s %s %s %s' % (x,y,object_list[object_index-1],color_candidate[0][1],color_candidate[0][2],color_candidate[0][3]))
		else:
			if(b < object_dic[object_list[object_index-1]][1]):
				temp,x = surf_candidate[1][1].split('=')
				temp,y = surf_candidate[1][2].split('=')
				pub_show.publish('%s %s %s %s %s %s' % (x,y,object_list[object_index-1],color_candidate[1][1],color_candidate[1][2],color_candidate[1][3]))
		surf_candidate = []
		color_candidate = []
		pub_main.publish('data')
	elif(candidate_index < len(surf_candidate)):
		print surf_candidate[candidate_index]
		temp,x = surf_candidate[candidate_index][1].split('=')
		temp,y = surf_candidate[candidate_index][2].split('=')
		
		pub_pointCheck.publish(str(int(x))+','+str(int(y)))
		candidate_index += 1
		

def surfCallback(data):
	global surf_candidate,candidate_index
	print data.data
	candidates = data.data.strip().split('|')
	for candidate in candidates:
		temp = candidate.strip().split(' ')
		surf_candidate.append(temp)
	candidate_index = 0
	decision('surf')

def color_histCallback(data):
	global color_candidate
	temp = data.data.strip().split(' ')
	color_candidate.append([temp[2],temp[3],temp[4],temp[5]])
	decision('color_hist')

def mainCallback(data):
	global pub_filename,object_index,object_list,surf_candidate,color_candidate,pub_extract
	if('reset' in data.data):
		print 'start extract'
                pub_extract.publish('start')

def countCallback(data):
	count = int(data.data)
	print 'object : ' + str(count)
        for i in range(0,count):
		for j in object_list:
			print i,j
			pub_filename.publish('%s %s' % ('object_'+str(i)+'.jpg','object_japan/'+j))

def control_object():
	global pub_filename,pub_main,pub_pointCheck,pub_show,pub_extract
	rospy.init_node('control_object')
	rospy.loginfo('Start control_object')
	pub_filename = rospy.Publisher('/object/filename',String)
	pub_show = rospy.Publisher('/object/show',String)
	pub_main = rospy.Publisher('/object/start_search',String)
	pub_pointCheck = rospy.Publisher('/object/pointcheck',String)
        pub_extract  = rospy.Publisher('/object/extract',String)
	rospy.Subscriber("/object/surf",String,surfCallback)
	rospy.Subscriber("/object/color_hist",String,color_histCallback)
	rospy.Subscriber("/object/start_search",String,mainCallback)
	rospy.Subscriber("/object/count",String,countCallback)
	rospy.spin()

if __name__ == '__main__':
	try:
		f = open('objectlist.txt','r')
		for line in f:
			temp = line.strip().split(' ')
			if('#' in temp[0]): continue
			object_list.append(temp[0])
			object_dic[temp[0]] = [temp[1],float(temp[2])]
		f.close()
		control_object()
	except:
		rospy.loginfo('Error Readfile')
		pass
