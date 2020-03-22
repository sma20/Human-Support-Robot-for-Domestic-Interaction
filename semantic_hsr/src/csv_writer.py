#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import numpy
import pandas as pd
from geometry_msgs.msg import PointStamped
_CONNECTION_TIMEOUT = 10.0
global M
data_file='../data/semantic_map.csv'
room_file='../data/room.csv'
M = [[0 for j in range(0,5)] for i in range(0,1)]
M[0]=['label','x','y','z','room']
room=[[0 for j in range(0,5)] for i in range(0,1)]

def _csv_(msg):
	#global M
	global data_file
	u=0
	M=[]
	room=[]
	#--------------opening the 2 csv file----------------------
	with open(data_file) as csvfile:
    		reader = csv.reader(csvfile) # change contents to floats
    		for row in reader: # each row is a list
        		M.append(row)
			u=u+1
	csvfile.close
	with open(room_file) as roomfile:
    		reader1 = csv.reader(roomfile) # change contents to floats
    		for row in reader1: # each row is a list
        		room.append(row)
			
	roomfile.close
	newrow = [msg.header.frame_id,msg.point.x,msg.point.y,msg.point.z,'Home']
	check=0
	index=0
	#--------Room assignement-------------
	for index in range(1,6):
		#print(room[index][1]-newrow[1])
		if(float(room[index][1])<newrow[1] and float(room[index][2])>newrow[1] and float(room[index][3])<newrow[2] and float(room[index][4])>newrow[2]):
			print('bob')
			newrow[4]=room[index][0]
	#--------Check if the object already exist--------------------
	for i in range(1,u):
		if(newrow[0]==M[i][0] and abs(newrow[1]-float(M[i][1]))<0.5 and abs(newrow[2]-float(M[i][2]))<0.5 and abs(newrow[3]-float(M[i][3]))<0.5):
			M[i][:]=newrow
			data=pd.DataFrame(M)
			data.to_csv(data_file, index=False, header=False)
			df=pd.read_csv(data_file)
			check=1
	#------------Put the new object in the semantic map---------------------
	if(check==0):
		M = numpy.vstack([M, newrow])
		data=pd.DataFrame(M)
		data.to_csv(data_file, index=False, header=False)
		df=pd.read_csv(data_file)
		u=u+1
				
		
	print("integration in csv of :",msg.header.frame_id," --Done--")		
		
	
	
sub=rospy.Subscriber('/3d_position_label', PointStamped, _csv_)
rospy.init_node('csv_writer')

try:
        rospy.spin()
except rospy.ROSException as e:
        rospy.logerr(e)
