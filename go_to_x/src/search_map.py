#!/usr/bin/env python
#NOTE: if using an action to move REAL ROBOT. need to do "initialization self-position of the robot"


#Brieuc: there may be a prob with the camera...the robot may be facing down, prob to detect anything on the tables. 
#option 1: we check for objects while the robot turn (a function to move head to add)
#option 2:...we try and move the head while it moves. we have to search how to do that in going deep into how action work 

import rospy, time
import roslib
import math as math
import actionlib
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from go_to_x.srv import find_goals, find_goalsResponse
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler#


POS_TOLERANCE=0.1
max_duration=60*2 #max duration before considering something went wrong: 2min
STOP=False

#----------------------- ODOM CLASS -------------------
#each time the class is called, the odom is updated
class odometry:
    def  __init__(self):
        """to call the odometry"""
        self.sub = rospy.Subscriber('/hsrb/odom', Odometry, self.getPose)
        self.poseX=0
        self.poseY=0
        self.posew=0
    
    def getPose(self,msg):
        """to get all the pose we need as part of the class"""
        self.poseX,self.poseY, self.posew = msg.pose.pose.position.x,msg.pose.pose.position.y, msg.pose.pose.orientation.w
        return self.poseX, self.poseY, self.posew

class getodom (odometry):
    def get(self):
        return self.poseX, self.poseY, self.posew

#------------------- TIME CHECK FCT ------------------------

#Check the whole process time hasn't extended the maximum duration allowed

#This fct is not complete
def check_stop(event):
	global begin_time
	global max_duration
	global STOP
	#print begin_time
	ros_time_now = rospy.Time.from_sec(time.time())
	now=ros_time_now.to_sec()
	#print now

	if (now-begin_time>max_duration):
		print "execution TOO LONG"
		STOP= True
		#print 'Timer called at ' + str(event.current_real)

#---------------------- TURN HSR FCT ------------------------------
def publish_once_in_cmd(speed):
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems, this is no big deal, but in systems that publish only
    once, it IS very important.
    """
    pub= rospy.Publisher("/hsrb/command_velocity_teleop",Twist,queue_size=4)

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)
    print ("Connected")
    pub.publish(speed)

    #rospy.loginfo("Cmd Published")

#not to miss any objects around. 	 
def turn():
    odom=getodom()
    """we do a 360
    """
    
    #make the robot turn 360 then move straight ahead 
    posex, posey, posew= odom.get()
    while abs(posew)>0.5 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        posex, posey, posew= odom.get()
        print posew
        #duration+=1

    while abs(posew)<0.98 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        posex, posey, posew= odom.get()
        #print posew
        #duration+=1

    rospy.sleep(0.5)

#------------------------------------------------------------------


#--------------------------------ACTION_MOVE --------------------------

#want to try and lift the head here (to see the objects well)
def check_result(cli):
    #if (!cli.isActive())  
    return 0    
    

def move_action(destination_x,destination_y, poseX,poseY):
    global POS_TOLERANCE
    global STOP
    goodTermination= False
    rate = rospy.Rate(2)

    # initialize action client
    cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    # wait for the action server to establish connection
    cli.wait_for_server()


    goal_begin=PoseStamped()
    
    initial_x = poseX
    initial_y = poseY
    initialDistance = math.sqrt((destination_x - initial_x)**2 + (destination_y - initial_y)**2)
    destination_angle = math.atan2(destination_y - initial_y, destination_x - initial_x)

    #the action takes in a posestamped
    print "destinationx, y and angle: ", destination_x, destination_y, destination_angle
    goal_begin.header.stamp = rospy.Time.now()
    goal_begin.header.frame_id = "map"
    goal_begin.pose.position=Point(destination_x,destination_y, 0)
    quat = quaternion_from_euler(0, 0, destination_angle)
    goal_begin.pose.orientation = Quaternion(*quat)
    
    goal = MoveBaseGoal()
    goal.target_pose = goal_begin

    # send message to the action server
    cli.send_goal(goal)

    # wait for the action server to complete the order
    cli.wait_for_result()

    #the result send a number corresponding to success or failure or what happened. 4: path avorted (something similar) 3:ok 0:ok? 
    #check_result(cli)

    #if delay passed we stop the robot here and there with the goal topic.
    if STOP== True:
        poseX,poseY = odom.getPose()
        print("actual pose", poseX, poseY)
        response_move = move_action(poseX, poseY, poseX, poseY)   
        STOP=False
        #rospy.sleep(2)
        
    # print result of navigation
    action_state = cli.get_state()
    if action_state == GoalStatus.SUCCEEDED:
        goodTermination=True
        print("goal reached")
    

    return goodTermination


#------------------------- NODE ----------------------------

pub = rospy.Publisher('job_done', Bool, queue_size=2)
#this publisher will allow us to monitor the state of this whole function
rospy.init_node('search_map', anonymous=True)
pub.publish(False) #once everything terminated, STOP


global begin_time
global STOP
odom= getodom()

#----------------- start identify goal service ----------------------
rospy.loginfo('Executing get goals to search room , service on?')
try:
	rospy.wait_for_service('/identify_goals')# Wait for the service to be running (with launch file)
	find_goals_service = rospy.ServiceProxy('/identify_goals',find_goals) # Create connection to service
	find_goals_request = find_goalsRequest() # Create an object of type EmptyRequest
except rospy.ServiceException, e:
	print ("Service get goals to search room failed: %s"%e)

rospy.loginfo('Executing service ')

#Testing purpose
"""
Sunbul: subscribe to the speech topic to retrieve romm and object name
to add here and change the find_goals_request.room by it
"""

find_goals_request.start_check=True
find_goals_request.room="kitchen"
#this service gives back 2 arrays (x,y) of several goals to go to in the room/home
response_goals= find_goals_service(find_goals_request)


#----------------- start move action ----------------------
#print(response_goals)

rospy.Timer(rospy.Duration(10), check_stop)#Check every 10s if time limit reached or stop message received

#we go from 1 position to the next
for i in range(len(response_goals.goals_to_reachx)):   
    #This begin_time is used to ensure that after a delay the action is stopped
    begin_time = (rospy.Time.from_sec(time.time())).to_sec()

    #check sake
    print("next goal:",response_goals.goals_to_reachx[i],response_goals.goals_to_reachy[i])
    #check odom  
    poseX,poseY, posew = odom.get()
    #send goal to move action  
    response_move = move_action(response_goals.goals_to_reachx[i],response_goals.goals_to_reachy[i], poseX, poseY)    
    turn()    
    
    """
    add here a check for the object in question.

    Brieuc: search in csv file object in this room/ or check yolo output directly (may be faster to just check labels?) 
    """


print ("end")
pub.publish(True) #to stop the launch file

