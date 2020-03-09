#! /usr/bin/env python


#NB: first tests won't be good because of the dimension of the robot (platform size)
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from go_to_x.srv import set_goal, set_goalResponse
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist 
import numpy as np

#to check if correct
import matplotlib.pyplot as plt
import numpy as np


resolution = 0.05	# resolution of the image in m/px
offsetX = -7 #instead of -15		# offset of the reduced map
offsetY = -7 #-15
fullSizeX = 2048 #4000	# Size in pixel of the map 4000px <-> 200m
fullSizeY = 2048 #4000
reducedSizeX = 600	# size of the map once reduced in px 600px <-> 30m
reducedSizeY = 600
cell_size = 4 #2 = 10cm/side 4= 20cm/side


poseX=0
poseY=0
posew=0
endService = False

# Convert the data from from the /map topic to a 2 dimensionnal array -> return only the reduced map
def convert_1D_to_2D(data1D):
	grid = np.zeros((reducedSizeX,reducedSizeY))
	x,y = fullSizeX/2-reducedSizeX/2,fullSizeY/2-reducedSizeY/2		# The starting coordinates (0,0) is at the center of the map.
	for i in range(reducedSizeX*reducedSizeY):
		grid[i%reducedSizeX][int(i/reducedSizeY)] = data1D[x+y*fullSizeX]
		x += 1
		if x >= fullSizeX/2+reducedSizeX/2:
			x = fullSizeX/2-reducedSizeX/2
			y += 1
	return grid

    # Convert the unit of a map from pixel to m
# cell_size represents the number of pixel that form a cell (ex: if a cell is composed of 8x8 pixels, cell_size = 8) 
def convert_px_to_xy(x_px,y_px):
	x = x_px * resolution*cell_size + offsetX
	y = reducedSizeY*resolution - y_px * resolution*cell_size + offsetY
	return x,y

def convert_xy_to_px(x,y):
	x_px = int((x-offsetX)/(resolution*cell_size))
	y_px = int((y-offsetY)/(resolution*cell_size))
	return(x_px,y_px)


# Returns the free cells of the map
def getFreeCells(grid):
	x,y = [],[]
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			if grid[i][j] == 0:	# Free cells <-> =0
				x.append(i)
				y.append(j)
	return x,y

# Returns the occupied cells of the map
def getOccupiedCells(grid):
	x,y = [],[]
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			if grid[i][j] > 0:	# Occupied pixels has a value of 100. As this function can work with cells composed of numerous pixels, we take all the values > 0.
				x.append(i)
				y.append(j)
	return x,y

# Create a coast map from the complete map
# It regroups pixels to form a bigger cell -to reduce the time lenght while reading each cell- 
def create_coastmap(grid):  
	coastmap = np.zeros((len(grid)/cell_size,len(grid[0])/cell_size))
	cursorX,cursorY = -1,-1
	for i in range(len(grid)):
		if i%cell_size == 0:
			cursorX += 1
		for j in range(len(grid[0])):
			if j%cell_size == 0: 
				cursorY += 1
			if cursorY >= len(coastmap):
				cursorY = 0
			coastmap[cursorX][cursorY] += grid[i][j]
	return coastmap



def getPose(msg):
	global poseX
	global poseY
	global posew
	poseX,poseY, posew=msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.w
    #the orientation will also be important, we

def find_goal(grid,goalx,goaly):

	global poseX
	global poseY
	global retry
	distance_map = np.zeros((len(grid),len(grid[0])))	# create the map which takes the distance values from the start point. All the non calculated cells are set at 0.
	distance_map[goalx][goaly] = -1	# Initialize the start at -1
	print("in")
	position_goal= Point()
	upx=0
	upy=0
	if poseX< goalx:
		upx=1
	if poseY< goaly:
		upy=1

	lastDistX = [goalx]
	lastDistY = [goaly]#[value]
	
	distance=0
	expand=-1
	next_goal=10

	free_cell=True
	posX, posY = [],[]
	goal_found=False
	

	while goal_found != True:
		distance+=1
		expand+=1
		# For each cells with the highest distance:
		for i in range(len(lastDistX)+retry+expand): #so at the beginning 1+retry (if first pose failed to be reached), *2 because 10cm not enough
			# Checks the 8 cells around each cell
			for m in range(3):
				for n in range(3):
					
					if grid[lastDistX[i]+m-1][lastDistY[i]+n-1]== 0 and distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] == 0:
						distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = distance
						posX.append(lastDistX[i]+m-1)
						posY.append(lastDistY[i]+n-1)

						if distance>retry:
							goal_found=True
 
					else:
						if grid[lastDistX[i]+m-1][lastDistY[i]+n-1] > 0 or grid[lastDistX[i]+m-1][lastDistY[i]+n-1] <0 : #in case we see through the arena
							distance_map[lastDistX[i]+m-1][lastDistY[i]+n-1] = len(grid)*len(grid[0])+1
						#give high value to cell

						# if cell not already filled, put the value of the distance to the cell in the distance_map, then ad the coordinate of the cell in pos

		print ("zone")
		print ("-------------------------------------------------")
		for i in range(goalx-m*expand+1,goalx+m*expand+1):#startY,goalX):
			for j in range(goaly-n*expand+1,goaly+n*expand+1):#startX,goalX):
				print int(distance_map[j][i]),
			print(" ")
		lastDistX = posX
		lastDistY = posY
		### END WHILE ###

	print("potential_posX",posX)
	print("potential_posY",posY)

	"""
	print ("zone")
	print ("-------------------------------------------------")
	for i in range(0,goalx):#startY,goalX):
		for j in range(0,goaly):#startX,goalX):
			print int(distance_map[j][i]),
		print(" ")
	"""
#to upgrade later on, here always come in a diagonal
	if upx ==1:
		position_goal.x=min(posX)
	else:
		position_goal.x=max(posX)

	if upy ==1: 
		position_goal.y=min(posY)
	else:
		position_goal.y=max(posY)

	if upy==1 and upx==1:
		position_goal.z=np.pi
	if upy==0 and upx==0:
		position_goal.z=0

	return position_goal, upx, upy






def callback(msg):
	global real_goal_position
	global position_goal
	global start_set_goal
	print("testin")
	if start_set_goal ==True: #double security
		print "start"
		grid_px = msg.data
		grid_2D = convert_1D_to_2D(grid_px)
		coastmap = create_coastmap(grid_2D) #new map
		#freeCells = getFreeCells(coastmap)
		#scaled = convert_px_to_xy(freeCells)
		print("actual pose x and y:",poseX,poseY)
		goalx,goaly=real_goal_position.x,real_goal_position.y
		print("goal position: ", goalx,goaly)
		goalx_px,goaly_px=convert_xy_to_px(goalx,goaly)

		position_goal,upx,upy = find_goal(coastmap,goalx_px,goaly_px)
		
		
		position_goal.x,y=convert_px_to_xy(position_goal.x,position_goal.y)
		print "done"
		position_goal.y=-y
		if upx==1:
			position_goal.x=position_goal.x-0.1 #take 10 cm off, to check with robot diameter 
		else: 
			position_goal.x=position_goal.x+0.1 #add 10 cm on, to check with robot diameter 
		if upy==1:
			position_goal.y=position_goal.y-0.1 #take 10 cm off, to check with robot diameter 
		else: 
			position_goal.y=position_goal.y+0.1 #add 10 cm on, to check with robot diameter 
		print position_goal.x,position_goal.y

		#print convert_xy_to_px(0.6,0.65)

		plt.scatter(freeCells[0],freeCells[1])
		plt.scatter(occupiedCells[0],occupiedCells[1],'r')
		#plt.scatter(unknownCells[0],unknownCells[1],'b')
		plt.grid(which='major',linestyle='-', alpha=0.5)
		plt.minorticks_on()
		plt.grid(which='minor', linestyle='-', alpha=0.5)
		#plt.show()    


# we call set goal (closest accessible point to an object) as a service so it's not running all the time, beside it is so fast that it's not detrimental to the rest of the structure
def service_callback(request): 
	global endService
	global real_goal_position
	global position_goal
	global start_set_goal
	global retry

	real_goal_position=request.real_goal_position
	start_set_goal=request.start_set_goal
	retry=request.retry
	if start_set_goal==True:
		doneOnce = False
		rospy.loginfo("set goal service ongoing")
		if doneOnce == False: #if we don't force the subscriber to be called only once they will keep being called again and again
			sub = rospy.Subscriber('/map', OccupancyGrid, callback)
			sub2 = rospy.Subscriber('/hsrb/odom', Odometry, getPose)
		doneOnce = True
		while endService == False: #we wait for the service to be executed fully before sending back info
			i = 0

		response = set_goalResponse()
		response.position_goal=position_goal
		response.success=True #to change depending on situation later
		start_set_goal=False
		print("end of set_goal service")
		return response  #to where?


rospy.init_node('set_closest_goal', anonymous=True)
service = rospy.Service('/set_goal', set_goal , service_callback)

rospy.spin()
