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

SCALE_FACTOR = 4 #cell_size = 4 #2 = 10cm/side 4= 20cm/side

poseX=0
poseY=0
posew=0
endService = False



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

def processOccupancyGrid(gridMessage, scaleFactor, cacheEmptyCells):
    grid = OccupancyGrid(gridMessage.data, gridMessage.info.height, gridMessage.info.width, gridMessage.info.resolution,
                (gridMessage.info.origin.position.x, gridMessage.info.origin.position.y))
    grid.scale(scaleFactor, cacheEmptyCells=cacheEmptyCells)
    grid.expandObstacles()

    return grid


def getPose(msg):
	global poseX
	global poseY
	global posew
	poseX,poseY, posew=msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.w
    #the orientation will also be important, we

#Gets the centroid of the largest frontier
def getCentroid(grid_px,goalx,goaly):
    global grid

    print "Received centroid request."

    grid = processOccupancyGrid(grid_px, SCALE_FACTOR, True)

    foundCentroid = False

    visited = set()
    clusters = []

    print "Started to look for clusters..."

    for cell in grid.empty:
        cluster = []

        # rospy.logdebug("---------> New cluster!")

        expandCluster(cell, cluster, visited)

        if len(cluster) != 0:
            clusters.append(cluster)
            # rospy.logdebug("---------> Cluster was added!")
        # else:
        #     rospy.logdebug("---------> Cluster was not added!")

    print "DONE"

    print "Calculating centroid..."

    centroidPos = Point()

    if len(clusters) != 0:
        #Sort clusters by the number of elements in them in order to check the largest clusters first
        clusters.sort(key = lambda tup: len(tup), reverse=True)

        for currentCluster in clusters:
            centroid = calculateCentroid(currentCluster)
            centroidValue = grid.getCellValue(centroid)

            try:
                #if the centroid is inside the unexplored area or obstacle, then find the closest empty cell
                if centroidValue == CellType.Obstacle or centroidValue == CellType.Unexplored:
                    path = PathFinder.findPathToCellWithValueClosestTo(grid, centroid, [CellType.Empty], (req.currentPos.x, req.currentPos.y))
                    centroid = path.pop(len(path) - 1)

                #if the centroid is on the border with the unexplored cell, then find the first empty cell that is not
                centroid = PathFinder.findTheFirstCellNotSurroundedWith(grid, centroid, CellType.Empty, [CellType.Unexplored])

                #Check if get trajectory will throw an error
                centroidPos = convertCellToPoint(centroid, grid.cellOrigin, grid.resolution)

	return position_goal, upx, upy






def callback(msg):
	global real_goal_position
	global position_goal
	global start_set_goal
	print("testin")
	if start_set_goal ==True: #double security
		print "start"
		grid_px = msg.data
		#grid_2D = convert_1D_to_2D(grid_px)
		#coastmap = create_coastmap(grid_2D) #new map
		#freeCells = getFreeCells(coastmap)
		#scaled = convert_px_to_xy(freeCells)
		print("actual pose x and y:",poseX,poseY)
		goalx,goaly=real_goal_position.x,real_goal_position.y
		print("goal position: ", goalx,goaly)
		goalx_px,goaly_px=convert_xy_to_px(goalx,goaly)

		position_goal,upx,upy = find_goal(grid_px,goalx_px,goaly_px)
		
		
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
