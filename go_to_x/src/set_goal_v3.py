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



SCALE_FACTOR = 4 #cell_size = 4 #2 = 10cm/side 4= 20cm/side


poseX=0
poseY=0
posew=0
endService = False



#-------------------------------------------------------GRID CLASS------------------------------------------------------------
#A class that has a function of cell type enumeration.
class CellType:
    Unexplored = -1
    Empty = 0
    Obstacle = 100

#Class that implements priority queue.
class PriorityQueue:
    #Initializes priority queue
    def __init__(self):
        self.elements = []

    #Checks if the queue is empty
    def empty(self):
        return len(self.elements) == 0

    #Puts the item with the given priority into the priority queue
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    #Pops the first item off the priority queue
    def get(self):
        return heapq.heappop(self.elements)[1]



#Class that implements grid.
class Grid:
    #Initializes the grid
    def __init__(self, data, height, width, resolution, origin):
        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin = origin

        (x, y) = origin
        self.cellOrigin = (x + resolution/2, y + resolution/2)

        self.data = []

        counter = 0
        previous_index = 0
        current_index = 0

        #convert 1 dimensional array to 2 dimensional array
        while current_index < len(data):
            counter += 1
            current_index = counter * width
            self.data.append(data[previous_index:current_index])
            previous_index = current_index

    #Gets the value of the cell from the grid
    def getCellValue(self, cell):
        (x, y) = cell

        return self.data[y][x]

    #Sets the value of the cell on the grid
    def setCellValue(self, cell, cellValue):
        (x, y) = cell

        self.data[y][x] = cellValue

    #Checks if cell coordinate is valid
    def checkIfCellValid(self, cell):
        if cell is not self.hasProperTypeAndSize(cell):
            raise Exception("The cell should be tuple consisting of two elements!")

        if cell is not self.isWithinGrid(cell):
            raise Exception("The cell should be within grid!")

    #Checks if a given cell is a tuple that consists of 2 elements
    def hasProperTypeAndSize(self, cell):
        if cell is not tuple or len(cell) != 2:
            return False
        else:
            return True

    #Checks if a given cell is within the current grid
    def isWithinGrid(self, cell):
        (x, y) = cell

        if x >= 0 and x < self.width - 1 and y >= 0 and y < self.height:
            return True
        else:
            return False

    #Prints the grid (primarily used for debugging).
    def printToConsole(self):
        for i in range(0, self.height):
            for j in range(0, self.width):
                cell = (j, self.height - 1 - i)
                print self.getCellValue(cell),
            print ""

    #Abstract method that scales the map by the scaleFactor.
    def scale(self, scaleFactor):
        raise NotImplementedError("scale() method is not implemented!")

    #Populates the set with cells that have the given value
    @staticmethod
    def populateSetWithCells(grid, set, value):
        for i in range(0, grid.height):
            for j in range(0, grid.width):
                cell = (j, i)
                cellType = grid.getCellValue(cell)
                if cellType == value:
                    set.add(cell)



#Class that implements occupancy grid
class OccupancyGridclass(Grid):
    #Initializes occupancy grid
    def __init__(self, data, height, width, resolution, origin):
        Grid.__init__(self, data, height, width, resolution, origin)

        #Creates the sets for obstacles and empty cells (for faster obstacle expansion and frontier searching)
        self.obstacles = set()
        self.empty = set()

        self.costGrid = None

    #Gets neighbors of the specific cell
    def getNeighbors(self, cell, includeObstacles=False):
        neighborList = []

        (x, y) = cell

        for i in range(0, 3):
            for j in range(0, 3):
                neighborCell = (x - 1 + j, y - 1 + i)

                if includeObstacles:
                    if self.isWithinGrid(neighborCell) and cell != neighborCell:
                        neighborList.append(neighborCell)
                else:
                    if self.isWithinGrid(neighborCell) and cell != neighborCell and self.getCellValue(neighborCell) != CellType.Obstacle:
                        neighborList.append(neighborCell)

        return neighborList

    #Gets neighbors of the specific cell + the sum of xy of each cell
    def getNeighbors2(self, cell, includeObstacles=False):
        neighborList = []
        sumcellList=[]

        (x, y) = cell

        for i in range(0, 3):
            for j in range(0, 3):
                neighborCell = (x - 1 + j, y - 1 + i)
                sumcell=(x + j -1 +y + i -1)
                if includeObstacles:
                    if self.isWithinGrid(neighborCell) and cell != neighborCell:
                        sumcellList.append(sumcell)
                        neighborList.append(neighborCell)
                else:
                    if self.isWithinGrid(neighborCell) and cell != neighborCell and self.getCellValue(neighborCell) != CellType.Obstacle:
                        neighborList.append(neighborCell)
                        sumcellList.append(sumcell)

        return neighborList, sumcellList
    #Gets neighbors of the specific cell in a specific range
    def getNeighborsfromcenter(self, cell, nrange, includeObstacles=False):
        neighborList = []
        sumcellList=[]


        (x, y) = cell

        for r in range (0,nrange):
            for i in range(0, 3 +nrange):
                for j in range(0, 3 +nrange):
                    neighborCell = ((x + j -1-nrange), (y + i -1-nrange))
                    sumcell=(x + j -1-nrange +y + i -1-nrange)
                    if includeObstacles:
                        if self.isWithinGrid(neighborCell) and cell != neighborCell:
                            neighborList.append(neighborCell)
                            sumcellList.append(sumcell)
                    else:
                        if self.isWithinGrid(neighborCell) and cell != neighborCell and self.getCellValue(neighborCell) != CellType.Obstacle:
                            neighborList.append(neighborCell)
                            sumcellList.append(sumcell)

        return neighborList, sumcell

    #Gets neighbors of the specific cell
    def getNeighborValues(self, cell):
        neighbors = self.getNeighbors(cell, True)
        neighborValues = []

        for neighbor in neighbors:
            neighborValue = self.getCellValue(neighbor)
            neighborValues.append(neighborValue)

        return neighborValues

    #Gets neighbors and their respective values as list of tuples
    def getNeighborValuePairs(self, cell):
        neighbors = self.getNeighbors(cell, True)
        neighborValuePairs = []

        for neighbor in neighbors:
            neighborValue = self.getCellValue(neighbor)
            neighborValuePair = (neighbor, neighborValue)

            neighborValuePairs.append(neighborValuePair)

        return neighborValuePairs

    #Scales map to a new resolution
    def scale(self, scaleFactor, cacheEmptyCells=True, cacheObstacleCells=True):
        self.obstacles.clear()
        self.empty.clear()

        if type(scaleFactor) != int:
            raise Exception("The scale factor should be an integer!")

        if scaleFactor < 1:
            raise Exception("New resolution should be larger than the old resolution!")

        ng_data = [] #ng stands for NewGrid
        ng_resolution = self.resolution * scaleFactor

        #Round up the new width and height
        ng_width = -(-self.width // scaleFactor)
        ng_height = -(-self.height // scaleFactor)

        ng_row = -1

        skip = False

        for i in range(0, self.height):
            temp_ng_row = i // scaleFactor

            #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
            if ng_row != temp_ng_row:
                ng_row = temp_ng_row
                ng_data.append([])

            ng_column = -1

            for j in range(0, self.width):
                temp_ng_column = j // scaleFactor

                #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
                if ng_column != temp_ng_column:
                    ng_column = temp_ng_column
                    ng_data[ng_row].append(-2) # -2 indicates that the new cell has no value assigned to it yet
                    skip = False

                if (ng_column, ng_row) in self.obstacles:
                    skip = True

                if skip:
                    continue

                currentCellValue = self.getCellValue((j, i))

                ng_oldCellValue = ng_data[ng_row][ng_column]

                if (currentCellValue == CellType.Obstacle):
                    ng_data[ng_row][ng_column] = CellType.Obstacle
                    if cacheObstacleCells:
                        self.obstacles.add((ng_column, ng_row))
                    if cacheEmptyCells and ng_oldCellValue == CellType.Empty:
                        self.empty.remove((ng_column, ng_row))

                elif (currentCellValue == CellType.Unexplored):
                    if ng_oldCellValue != CellType.Obstacle:
                        ng_data[ng_row][ng_column] = CellType.Unexplored
                        if cacheEmptyCells and ng_oldCellValue == CellType.Empty:
                            self.empty.remove((ng_column, ng_row))

                else: #empty cell
                    if ng_oldCellValue != CellType.Obstacle and ng_oldCellValue != CellType.Unexplored:
                        ng_data[ng_row][ng_column] = CellType.Empty
                        if cacheEmptyCells:
                            self.empty.add((ng_column, ng_row))

        self.data = ng_data
        self.height = ng_height
        self.width = ng_width
        self.resolution = ng_resolution

        (x, y) = self.origin
        self.cellOrigin = (x + ng_resolution/2, y + ng_resolution/2)

    #Expands the obstacles
    def expandObstacles(self):
        newObstacles = set()

        if not self.obstacles: #If the obstacle set is empty, then iterate through the entire map and find obstacles
            Grid.populateSetWithCells(self, self.obstacles, CellType.Obstacle)

        for obstacleCell in self.obstacles:
            neighborCells = self.getNeighbors(obstacleCell)

            for neighborCell in neighborCells:
                self.setCellValue(neighborCell, CellType.Obstacle)
                newObstacles.add(neighborCell)

        self.obstacles = self.obstacles.union(newObstacles)

        if self.empty: #If the empty cell cache is not empty, then remove empty cells that turned into obstacles after expansion
            self.empty = self.empty - self.obstacles

    #Adds cost grid to allow local cost map processing
    def addCostGrid(self, costGrid):
        self.costGrid = costGrid

        if (self.resolution != costGrid.resolution):
            raise Exception("Current implemenation does not support the addition of the cost grid that has different "
                            "resolution than the occupancy grid.")

        #Warning: The offset calculation
        self.costGridCellOffset = (int((costGrid.cellOrigin[0] - self.cellOrigin[0]) / self.resolution),
                                   int((costGrid.cellOrigin[1] - self.cellOrigin[1]) / self.resolution))

    #Gets heuristic relevant to the current grid
    def getHeuristic(self, currentCell, destinationCell):
        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        return math.sqrt((currentX - destinationX) ** 2 +
                         (currentY - destinationY) ** 2)

    #Gets heuristic relevant to the current grid
    def getHeuristic(self, currentCell, destinationCell):
        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        return math.sqrt((currentX - destinationX) ** 2 +
                         (currentY - destinationY) ** 2)

    #Gets path cost relevant to the current grid
    def getPathCost(self, currentCell, destinationCell):
        cost = 0

        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        xDiff = abs(currentX - destinationX)
        yDiff = abs(currentY - destinationY)

        if (xDiff > 1 or xDiff < 0) or (yDiff > 1 or yDiff < 0):
            raise Exception("getPathCost: The function estimates the cost only for adjacent cells!")

        if xDiff + yDiff == 2:
            cost += 1.4
        elif xDiff + yDiff == 1:
            cost += 1

        if self.costGrid != None:
            #find the corresponding cell in costGrid
            costGridCell = (currentCell[0] - self.costGridCellOffset[0], currentCell[1] - self.costGridCellOffset[1])

            #add the additional cost if the cell is in costmap
            if self.costGrid.isWithinGrid(costGridCell):
                cost += self.costGrid.getCellValue(costGridCell)

        return cost

#Class that implements cost grid
class CostGrid(Grid):
    #Initializes occupancy grid
    def __init__(self, data, height, width, resolution, origin):
        Grid.__init__(self, data, height, width, resolution, origin)

    #Scales the occupancy grid by the factor provided
    def scale(self, scaleFactor):
        if type(scaleFactor) != int:
            raise Exception("The scale factor should be an integer!")

        if scaleFactor < 1:
            raise Exception("New resolution should be larger than the old resolution!")

        ng_data = [] #ng stands for NewGrid
        ng_resolution = self.resolution * scaleFactor

        #Round up the new width and height
        ng_width = -(-self.width // scaleFactor)
        ng_height = -(-self.height // scaleFactor)

        #Since the cells are squares, then the maximum number of cells in the old grid per cell in the new grid is going
        #to be the square of the scaleFactor
        max_cells_per_ng_cell = scaleFactor ** 2

        ng_row = -1

        for i in range(0, self.height):
            temp_ng_row = i // scaleFactor

            #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
            if ng_row != temp_ng_row:
                ng_row = temp_ng_row
                ng_data.append([])

            ng_column = -1

            for j in range(0, self.width):
                temp_ng_column = j // scaleFactor

                #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
                if ng_column != temp_ng_column:
                    ng_column = temp_ng_column
                    ng_data[ng_row].append([0, 0]) # -2 indicates that the new cell has no value assigned to it yet

                cellValue = self.getCellValue((j, i))
                ng_cellValue = ng_data[ng_row][ng_column]

                ng_cellValue[0] += cellValue
                ng_cellValue[1] += 1

                [ng_sum, ng_count] = ng_cellValue

                #If the max count of cells per new grid cell was reached, then replace sum and count pair with the actual average value
                if (ng_count >= max_cells_per_ng_cell):
                    ng_data[ng_row][ng_column] = ng_sum/ng_count

        #Iterate through the last row and column to make sure all the sum and count pairs were converted into the actual average
        for i in range(0, ng_height):
            if (i == ng_height - 1):
                for j in range(0, ng_width):
                    ng_cellValue = ng_data[i][j]
                    if type(ng_cellValue) == list:
                        [ng_sum, ng_count] = ng_cellValue
                        ng_data[i][j] = ng_sum/ng_count
            else:
                ng_cellValue = ng_data[i][ng_width-1]
                if type(ng_cellValue) == list:
                    [ng_sum, ng_count] = ng_cellValue
                    ng_data[i][j] = ng_sum/ng_count

        self.data = ng_data
        self.height = ng_height
        self.width = ng_width
        self.resolution = ng_resolution

        (x, y) = self.origin
        self.cellOrigin = (x + ng_resolution/2, y + ng_resolution/2)

#----------------------------------------------------END GRID CLASS----------------------------------------------------

#get a redimensionned occupancygrid
def processOccupancyGrid(gridMessage, scaleFactor, cacheEmptyCells):
    grid = OccupancyGridclass(gridMessage.data, gridMessage.info.height, gridMessage.info.width, gridMessage.info.resolution,
                (gridMessage.info.origin.position.x, gridMessage.info.origin.position.y))
    grid.scale(scaleFactor, cacheEmptyCells=cacheEmptyCells)
    grid.expandObstacles()

    return grid

#Processes the received cost grid messsage.
def processCostGrid(gridMessage, scaleFactor):
    costGrid = CostGrid(gridMessage.data, gridMessage.info.height, gridMessage.info.width, gridMessage.info.resolution,
                (gridMessage.info.origin.position.x, gridMessage.info.origin.position.y))
    costGrid.scale(scaleFactor)
    return costGrid

#Callback function that processes the initial position received.
def convertPointToCell(goalx,goaly, gridOrigin, resolution):
    (gridOriginX, gridOriginY) = gridOrigin

    goalPosCellX = int((goalx - gridOriginX) // resolution)
    goalPosCellY = int((goaly- gridOriginY) // resolution)

    tempCell = (goalPosCellX, goalPosCellY)
    """
    if not grid.isWithinGrid(tempCell):
        raise Exception("Error: The position selected was outside of the grid! Please, try again.")
    """
    return goalPosCellX, goalPosCellY

#Converts cells to points
def convertCellToPoint(x,y, cellOrigin, resolution):

    (cellOriginX, cellOriginY) = cellOrigin

    
    posx = cellOriginX + resolution*x
    posy = cellOriginY + resolution*y

    return posx, posy


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



def getPose(msg):
	global poseX
	global poseY
	global posew
	poseX,poseY, posew=msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.w
    #the orientation will also be important, we


#find closest accessible point around goal. NOTe : could be interesting to check z of goal to know if obstacle
def find_goal(grid,goalx,goaly):
    global endService
    global poseX
    global poseY
    global retry
    goal_found=False
    position_goal=Point()
    pose_list=[]
    centroidValue = grid.getCellValue((goalx,goaly))
    
    print("goalx y in px:",goalx,goaly)

    if centroidValue == CellType.Empty: # to cast this cell out of the search (won't be considered empty)
        setCellValue((goalx,goaly), CellType.Obstacle)
    

    while goal_found==False:
        			# Checks the 8 cells around each cell
        goal_found=True

        if retry==0:
            pose_list,sumcellList = grid.getNeighbors2((goalx,goaly))
            print("poselist ", pose_list)
            if not pose_list : #if no goal found around position
                retry+=1
                print("poselist empty")
                goal_found=False

        if retry>0: #if this isn't our first try to find a goal
            print("retry",retry)
            new_pose_list=[]
            pose_list,sumcellList = grid.getNeighborsfromcenter((goalx,goaly), retry, False) 
            goal_found=True
            if not pose_list :
                retry+=1
                goal_found=False

	print("potential_posX",pose_list)
#to know from where we are coming
	upx=0
	upy=0
	if poseX< goalx:
		upx=1 #goal above current pose
	if poseY< goaly:
		upy=1 #goal above current pose
    
    print("upx,upy:",upx,upy)

    #to upgrade later on
    if upx ==1 and upy==1 :
        index=np.argmin(sumcellList)
        position_goal.x=pose_list[index][0]
        position_goal.y=pose_list[index][1]

    elif (upx ==0 and upy==0) :
        index=np.argmax(sumcellList)
        position_goal.x=pose_list[index][0]
        position_goal.y=pose_list[index][1]

    else :

        index=np.argmax(pose_list[:,1])
        #indexy=np.argmin(pose_list[0,:])
        position_goal.x=pose_list[index][0]
        position_goal.y=pose_list[index][1]

    if upy==1 and upx==1:
        position_goal.z=np.pi
    else:
    #if upy==0 and upx==0:
        position_goal.z=0

    print("position_goal in px:", position_goal)
    endService=True
    return position_goal, upx, upy






def callback(msg):
    global real_goal_position
    global position_goal
    global start_set_goal
    print("testin")
    if start_set_goal == True : #double security    
        print ("start")
        #grid_px = msg.data
        grid=processOccupancyGrid(msg, SCALE_FACTOR, True)
        coastmap= processCostGrid(msg, SCALE_FACTOR)
        #grid_2D = convert_1D_to_2D(grid_px)
        #coastmap = create_coastmap(grid_2D) #new map
        #freeCells = getFreeCells(coastmap)
        #scaled = convert_px_to_xy(freeCells)

        print("actual pose x and y:",poseX,poseY)
        goalx,goaly=real_goal_position.x,real_goal_position.y
        print("goal position: ", goalx,goaly)
        goalx_px,goaly_px= convertPointToCell( goalx,goaly, grid.origin, grid.resolution)
        #goalx_px,goaly_px=convert_xy_to_px(goalx,goaly)

        position_goal,upx,upy = find_goal(grid,goalx_px,goaly_px)

        position_goal.x,y=convertCellToPoint(position_goal.x,position_goal.y, grid.cellOrigin, grid.resolution)
        #position_goal.x,y=convert_px_to_xy(position_goal.x,position_goal.y)
        print "done"
        position_goal.y=y
        if upx==1:
            position_goal.x=position_goal.x-0.1 #take 10 cm off, to check with robot diameter 
        else: 
            position_goal.x=position_goal.x+0.1 #add 10 cm on, to check with robot diameter 
        if upy==1:
            position_goal.y=position_goal.y-0.1 #take 10 cm off, to check with robot diameter 
        else: 
            position_goal.y=position_goal.y+0.1 #add 10 cm on, to check with robot diameter 

        print ("position in xy:",position_goal.x,position_goal.y)

        #print convert_xy_to_px(0.6,0.65)

        #plt.scatter(freeCells[0],freeCells[1])
        #plt.scatter(occupiedCells[0],occupiedCells[1],'r')
        #plt.scatter(unknownCells[0],unknownCells[1],'b')
        #plt.grid(which='major',linestyle='-', alpha=0.5)
        #plt.minorticks_on()
        #plt.grid(which='minor', linestyle='-', alpha=0.5)
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
        print("give back response")
        response = set_goalResponse()
        response.position_goal=position_goal
        response.success=True #to change depending on situation later
        start_set_goal=False
        print("end of set_goal service")
        return response  #to where?


rospy.init_node('set_closest_goal', anonymous=True)
service = rospy.Service('/set_goal', set_goal , service_callback)

rospy.spin()
