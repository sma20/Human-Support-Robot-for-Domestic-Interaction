#!/usr/bin/env python
# YW-Ma /Corner_Detection 
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as image
from nav_msgs.msg import OccupancyGrid

scaleFactor=2 #resizing of the map to be able to work on it
pathmap= '/home/cata/hsr_ws2/src/go_to_x/map_tests/hector_first_lab_map.pgm'


def scale(map, resolution, height, width, x,y):
    global scaleFactor

    ng_data = [] #ng stands for NewGrid
    ng_resolution = resolution * scaleFactor

    #Round up the new width and height
    ng_width = -(- width // scaleFactor)
    ng_height = -(- height // scaleFactor)

    #Since the cells are squares, then the maximum number of cells in the old grid per cell in the new grid is going
    #to be the square of the scaleFactor
    max_cells_per_ng_cell = scaleFactor ** 2

    ng_row = -1

    for i in range(0, height):
        temp_ng_row = i // scaleFactor

        #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
        if ng_row != temp_ng_row:
            ng_row = temp_ng_row
            ng_data.append([])

        ng_column = -1

        for j in range(0,  width):
            temp_ng_column = j // scaleFactor

            #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
            if ng_column != temp_ng_column:
                ng_column = temp_ng_column
                #ng_data[ng_row].append(0) # -2 indicates that the new cell has no value assigned to it yet

            #ng_cellValue = ng_data[ng_row][ng_column]

            #here we convert the map scale (from -1 to 100) to a grayscale (0 to 250)
            """
            if(ng_data[ng_row][ng_column]==-1): #-1 : unknown area 
                ng_data[ng_row][ng_column] = 127 #grey pixels
            elif(ng_data[ng_row][ng_column]>0.65):# probably something there
                ng_data[ng_row][ng_column] = 0 #black pixel
            else:
                ng_data[ng_row][ng_column] = 255 #white pixels
            """
            ng_data[ng_row].append([0, 0]) # -2 indicates that the new cell has no value assigned to it yet

            cellValue = map[i][j]
            ng_cellValue = ng_data[ng_row][ng_column]

            ng_cellValue[0] += cellValue
            ng_cellValue[1] += 1

            [ng_sum, ng_count] = ng_cellValue

            #If the max count of cells per new grid cell was reached, we believe it will be an unknown area
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
    

    #(x, y) = self.origin
    cellOriginx, cellOriginy = x + ng_resolution/2, y + ng_resolution/2
    return ng_data,ng_resolution, cellOriginx, cellOriginy

# Convert the data from from the /map topic to a 2 dimensionnal array -> return only the reduced map
def rescale(data1D,resolution, height, width, xo,yo):
    #new resolution
    data1D = np.array(data1D)
    print(np.shape(data1D))
    ng_resolution = resolution * scaleFactor

    #Round up the new width and height
    ng_width = -(- width // scaleFactor)
    ng_height = -(- height // scaleFactor)

    #create a new empty grid
    grid = np.zeros((ng_width,ng_height))

    #get the new origin
    cellOriginx, cellOriginy = xo + ng_resolution/2, yo + ng_resolution/2
    x,y = (width-1)/2-ng_width/2,(height-1)/2-ng_height/2# The starting coordinates (0,0) is at the center of the map.

    for i in range((ng_width-1)*(ng_height-1)):

        #here we convert the map scale (from -1 to 100) to a grayscale (0 to 250)    
        grid[i%ng_width][int(i/ng_height)] =data1D[x][+y*(width-1)][0] #white pixels
        x += 1
        if x >= width/2+ng_width/2:
            x = width/2-ng_width/2
            y += 1

	return grid,ng_resolution, cellOriginx, cellOriginy


def callback(msg):
    global scaleFactor
    global pathmap
    print("in")
    angles_positions=[]
    img = cv2.imread(pathmap)
    

    grid,ng_resolution,cellOriginx, cellOriginy=rescale(img, msg.info.resolution ,msg.info.height, msg.info.width,
                msg.info.origin.position.x, msg.info.origin.position.y)
    #grid,ng_resolution,cellOriginx, cellOriginy=scale(img, msg.info.resolution ,msg.info.height, msg.info.width,
                                                    #msg.info.origin.position.x, msg.info.origin.position.y)
    #print grid
    
    cv2.imshow('corners',img)
    if cv2.waitKey(0) & 0xff == 27:
        cv2.destroyAllWindows()

    
    corners=cornerFinder_Harris(img,5,0.5,0.04,2,5) #mode 2 used
    #mark the result
    #1. get the location of corners
    shape=corners.shape
    row=shape[0]
    col=shape[1]
    corners=corners.reshape(row*col)
    corners=np.squeeze(corners)
    index=np.where(corners==1)
    index=np.array(index)
    index=np.squeeze(index)

    index_y=index//col
    index_x=index-index_y*col
    print("out")
    #2. mark corners with red circle, users can personalize the radius of them.
    for item in range(index.size):
        """
        x = index_x[item]*ng_resolution+ cellOriginx
        y = index_y[item]* ng_resolution+ cellOriginy
        """
        x = index_x[item]*resolution+ msg.info.origin.position.x
        y = index_y[item]* resolution+ msg.info.origin.position.y

        print("angle position in px:",x,y)
        angles_positions.append(x,y) 

    print("angles positions in m",angles_positions)




def cornerFinder_Harris(img,win_size=5,sigmaXY=0.5,k=0.04,mode=2,param=5):
    print("deeper")
    """
    Parameters:
        img: input gray image, grayscale(0-255)
        win_size: sliding window size
        sigmaXY: parameter of GaussianBlur model, set it to 0 to turn the model off; Both sigmaX and sigmaY are specified by sigmaXY.
        k: a constant in Harris corner detection. OpenCV tutorial set it to 0.04
        mode & param: corner extraction mode selection
            mode 1: sort all corners by Intensity, select the best points using threshold 'param';
            mode 2: choose the best one of every 'param' x 'param' local regions
    Return:
        A grayscale image(float32) which marks all the corners detected with 1.0f and the rest with 0.0f.
    """
    #1. Get the gradiant of the whole image
    gx,gy = np.gradient(img)
    print("1")
    print("gx,gy:",gx,gy)
    #3. Calculate Intensity pixelwisely. I = det(M)-k*tr(M); M is consisted of gx and gy
    det_M=(gx*gy)-(gx*gy*gx*gy)
    tr_M=gx+gy
    Intensity=det_M-k*tr_M*tr_M
    print("intensity:",Intensity)
    #4. 
    # mode 1: Sort all coners by Intensity, select the best points using threshold 'param';
    # mode 2: Choose the best one of every 'param' x 'param' local regions
    Intensity=np.abs(Intensity)
    if(mode!=1 and mode!=2):
        print('Invalid mode\n')
        print("2")
        return img
    
    elif(mode==1): #mode 1
        #normalize the Intensity
        Intensity=(Intensity-Intensity.min())/Intensity.max()
        mask=Intensity>param
        Intensity=np.zeros(Intensity.shape)
        Intensity[mask]=1
        print("3")
        return Intensity
    
    elif(mode==2): #mode 2
        Intensity=(Intensity-Intensity.min())/Intensity.max()
        shape_X,shape_Y = Intensity.shape
        mask=Intensity>0
        group_X=shape_X//param
        group_Y=shape_Y//param
        #for the main part
        for step_X in range(group_X-1):
            for step_Y in range(group_Y-1):
                pad=Intensity[(step_X)*param:(step_X+1)*param,(step_Y)*param:(step_Y+1)*param]
                mask[(step_X)*param:(step_X+1)*param,(step_Y)*param:(step_Y+1)*param]=(pad>0.001)*(pad==pad.max())
                
        #for the last line ((step_Y+1)*param will exceed the boundary)
        for step_X in range(group_X-1):
            pad=Intensity[(step_X)*param:(step_X+1)*param,(step_Y)*param:shape_Y]
            mask[(step_X)*param:(step_X+1)*param,(step_Y)*param:shape_Y]=(pad>0.001)*(pad==pad.max())
            
        #for the last column((step_X+1)*param will exceed the boundary)
        for step_Y in range(group_Y-1):
            pad=Intensity[(step_X)*param:shape_X, (step_Y)*param:(step_Y+1)*param]
            mask[(step_X)*param:shape_X, (step_Y)*param:(step_Y+1)*param]=(pad>0.001)*(pad==pad.max())
            
        #to the last block
        pad=Intensity[step_X*param:shape_X,step_Y*param:shape_Y]
        mask[step_X*param:shape_X,step_Y*param:shape_Y]=(pad>0.001)*(pad==pad.max())
        
        Intensity=np.zeros(Intensity.shape)
        Intensity[mask]=1
        print("end1")
        return Intensity



# corners = cornerFinder_Harris(gray,5,0.5,0.04,1,0.15) #--> uncomment it to use mode 1
#corners = cornerFinder_Harris(gray,5,0.5,0.04,2,5) # mode 2
if __name__ == "__main__":
    sub = rospy.Subscriber('/map', OccupancyGrid, callback)

    rospy.init_node('get_corners', anonymous=True)
    print("start")
    rospy.spin()
