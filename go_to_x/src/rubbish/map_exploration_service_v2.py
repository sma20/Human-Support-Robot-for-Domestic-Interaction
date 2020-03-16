#!/usr/bin/env python
"""
This function take as entry the room (or home) to explore and set n points as goal. 
We need to know the corners of the room.
Then we extracts the points in this room from the map and search 

+ and - : 
The rooms have to be rectangular.
It's fast
It doesn't see if there is objects like tables or what, only if it's occupied/unknown or not

"""
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as image
from nav_msgs.msg import OccupancyGrid
from go_to_x.srv import find_goals, find_goalsResponse


#---------------------SIMULATION ROOMS SIZES --------------------------------
"""
corridor1 [-1, -3.2] #topright (face to main door entry, while in the appartment)
corridor2 [0.5,-3.2 ]#top left
corridor3 [0.5,0]#bottom left
corridor4 [-1, 0]#bottom right

kitchen1 [0.5, 0]#angle
kitchen2 [5, 0]#top left almost end of room
kitchen3 [5,3.5]#bottom left
kitchen4 [-1, 3.5]#bottom right

bathroom1 [0.5,-3.2] #top right
bathroom2 [3.1,-3.2] #top left
bathroom3 [3.1,0] #bottom left
bathroom4 [0.5,0]#bottom right

bedroom1 [3.1, -3.2] #top right
bedroom2 [8,-3.2] #top left
bedroom3 [8,0]# bottom left
bedroom4 [3.1,0] #bottom right

living_room1 [5,0] #top right
living_room2 [8,0] #top left
living_room3 [8, 3.5] #bottom left
living_room4 [5, 3.5] #bottom right

home1 [-1, -3,2] #top right
home2 [8,-3.2] #top left
home3 [8, 3.5] #bottom left
home4 [-1, 3.5]#bottom right
"""
#------------------------ END SIMULATION ROOM SIZES ---------------------------------
goals_to_reach_xm,goals_to_reach_ym=[],[]
endService=False


#Convert from points to cells
def convertPointToCell(goalx,goaly, gridOriginX, gridOriginY, resolution):

    goalPosCellX = int((goalx - gridOriginX) / resolution)
    goalPosCellY = int((goaly- gridOriginY) / resolution)

    """
    if not grid.isWithinGrid(tempCell):
        raise Exception("Error: The position selected was outside of the grid! Please, try again.")
    """
    return goalPosCellX, goalPosCellY

#Converts cells to points
def convertCellToPoint(x,y, cellOriginX, cellOriginY, resolution):
 
    posx =  resolution*x + cellOriginX
    posy =  resolution*y + cellOriginY 
    return posx, posy

#here we set our goals accross the room/home in points where we believe there is no obstacles
def goals_points(xminc, xmaxc, yminc,ymaxc, matrix_map, resolution, gridOriginX, gridOriginY):
	goals_to_reach=[]
	goals_to_reachxm, goals_to_reachym=[],[]

#We check the whole room. +6 ==30cm to be sure we don't try to enters the angles
	for x in range(xminc+4, xmaxc):
		for y in range(yminc+4,ymaxc):  
			#print("data", map.data[x+y*map_sizeX])	
			#We want the area to be free of objects		
			if matrix_map[x][y]==0 and matrix_map[x-1][y]==0 and matrix_map[x][y-1]==0 and matrix_map[x-1][y-1]==0 and  matrix_map[x+1][y+1]==0 and matrix_map[x+1][y]==0 and matrix_map[x+1][y-1]==0 and matrix_map[x][y+1]==0 and matrix_map[x-1][y+1]==0: #the extremities shouldn't be a problem
				if matrix_map[x+2][y-1]==0 and matrix_map[x+2][y]==0 and matrix_map[x+2][y+1]==0 and matrix_map[x+2][y+2]==0 and matrix_map[x+1][y+2]==0 and matrix_map[x][y+2] ==0 and matrix_map[x-1][y+2]==0:
					#map_division[x-left][y-down]=0 #just to visualize it 
					if (len(goals_to_reach)== 0): #if there is already something in goals_to_reach, else prob 
						goals_to_reach.append([x, y])
						
					else:
						#we check we haven't already a goal set around those parts, to avoid goals too close from one another
						goal_too_close=False
						for i in range(len(goals_to_reach)):
							#8 == 40cm in world m
							if ((abs(y-goals_to_reach[i][1]) <9) or abs(x-goals_to_reach[i][0]) <9): #if we already have a goal around this pose
								goal_too_close=True
								break
						if (goal_too_close==False):
							goals_to_reach.append([x, y])
							#map_division[x-left][y-down]=5


	#print("goals_to reach")
	#print(goals_to_reach)
	cellOriginX, cellOriginY=  gridOriginX+resolution/2, gridOriginY+resolution/2
	print("goals_to_reach in px")
	print(goals_to_reach)
	#cellOriginX, cellOriginY=convertPointToCell(gridOriginX,gridOriginY, gridOriginX,gridOriginY,resolution)
	print(np.shape(goals_to_reach))
	for i in range(len(goals_to_reach)):
		xm,ym=convertCellToPoint(goals_to_reach[i][0],goals_to_reach[i][1], cellOriginX, cellOriginY, resolution)
		goals_to_reachxm.append(xm)
		goals_to_reachym.append(ym)
	return goals_to_reachxm, goals_to_reachym


#CALLBACK MAP SUB (in service callback)
def callback_map(map):
	print("tere")
	global endService
	global goals_to_reach_xm
	global goals_to_reach_ym
	map_sizeX=map.info.width
	map_sizeY=map.info.height
	resolution= map.info.resolution
	gridOriginX, gridOriginY= (map.info.origin.position.x), (map.info.origin.position.y)
	cellOriginX, cellOriginY=  gridOriginX+resolution/2, gridOriginY+resolution/2
	"""
	print(gridOriginX, gridOriginY)
	print(cellOriginX, cellOriginY)
	"""
	#list_map= list(map.data)

	#test with kitchen FOR THE MOMENT------------------------------------------------
	xmin=0.5
	xmax=5
	ymin=0
	ymax=3.5
	

	xminc,yminc=convertPointToCell(xmin,ymin, gridOriginX, gridOriginY, resolution)
	xmaxc,ymaxc=convertPointToCell(xmax,ymax, gridOriginX, gridOriginY, resolution)
	"""
	testx,testy=convertCellToPoint(xminc,yminc, cellOriginX, cellOriginY, resolution)
	print("xmin,ymin",xmin,ymin)
	print("xminc,yminc",xminc,yminc)
	print("testx,testy",testx,testy)
	"""

	matrix_map= []
	current_index = 0
	previous_index = 0
	counter=0

	#convert 1 dimensional array to 2 dimensional array
	while current_index < len(map.data):
		counter += 1
		current_index = counter * map_sizeY
		matrix_map.append(map.data[previous_index:current_index])
		previous_index = current_index

	
	#map_division = np.zeros((xlen,ylen))	# (x,y)
	goals_to_reach_xm, goals_to_reach_ym=goals_points(xminc, xmaxc, yminc,ymaxc,  matrix_map, resolution, gridOriginX, gridOriginY)
	print("goals to reach in m")
	for i in range(len(goals_to_reach_xm)):
		print(goals_to_reach_xm[i], goals_to_reach_ym[i])
	endService=True


def callback_room(data):
	global endService
	doneOnce = False
	start_check=data.start_check
	room=data.room

	response=find_goalsResponse()
	print("identify goals for room exploration is on")
    #retrieve room we want to search in (or home if any) from topic or whatever
    #retrieve xmin,ymax etc from csv file

	if doneOnce == False:
		print("test")
		sub = rospy.Subscriber('/map', OccupancyGrid, callback_map)
		print("retest")
		doneOnce=True
	while endService == False:
		a=0
	response.goals_to_reachx=goals_to_reach_xm
	response.goals_to_reachy= goals_to_reach_ym
	print("identify goals for room exploration finished")
	return response





if __name__ == "__main__":
    #sub = rospy.Subscriber('/map', OccupancyGrid, callback)

	rospy.init_node('identify_goals_search_map', anonymous=True)
	print("start")

	my_service = rospy.Service('/identify_goals', find_goals , callback_room)
	rospy.loginfo("check map Service Ready")


	rospy.spin()
