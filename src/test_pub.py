#!/usr/bin/env python
import rospy, time
import roslib
from std_msgs.msg import Bool
from go_to_x.srv import *
import math as math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler#
"""
poseX=0
poseY=0
"""
POS_TOLERANCE=0.1
max_duration=60*2 #max duration before considering something went wrong: 3min
STOP=False
#----------------------- ODOM CLASS -------------------
class odometry:
	def  __init__(self):
		"""to call the odometry"""
		self.sub = rospy.Subscriber('/hsrb/odom', Odometry, getPose)
		self.poseX
		self.poseY

	def getPose(self,msg):
		"""to get all the pose we need as part of the class"""

		self.poseX,self.poseY=msg.pose.pose.position.x,msg.pose.pose.position.y
		return self.poseX, self.poseY

#------------------- TIME CHECK FCT ------------------------

#Check the whole process time hasn't extended the maximum duration allowed
#NB: see with Sunbul if we can' add a check keyword "STOP" to stop everything directly here

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
		#exit=True
		#rospy.is_shutdown()
		#to decide what to do here
		"""
		if speech topic receive stop than stop robot and exit with a return stop
		"""
		#print 'Timer called at ' + str(event.current_real)

#---------------------- TURN HSR FCT ------------------------------

	 
def turn():
    """we do a 360
    """
    global posew
    #make the robot turn 360 then move straight ahead 
    call_pose()
    while abs(posew)>0.5 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        print posew
        #duration+=1

    while abs(posew)<0.98 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        #print posew
        #duration+=1

    rospy.sleep(0.5)

#------------------------------------------------------------------

#pub = rospy.Publisher('fake_pub', Bool, queue_size=1)
rospy.init_node('fake_pub', anonymous=True)
pub= rospy.Publisher()
"""
global poseX
global poseY
"""
global POS_TOLERANCE
global begin_time
global STOP
odom= odometry()
#----------------- start identify goal service ----------------------
rospy.loginfo('Executing get goals to search room , service on?')
try:
	rospy.wait_for_service('/identify_goals')# Wait for the service to be running (with launch file)
	find_goals_service = rospy.ServiceProxy('/identify_goals',find_goals) # Create connection to service
	find_goals_request = find_goalsRequest() # Create an object of type EmptyRequest
except rospy.ServiceException, e:
	print ("Service get goals to search room failed: %s"%e)

rospy.loginfo('Executing service ')
find_goals_request.start_check=True
find_goals_request.room="kitchen"
response_goals= find_goals_service(find_goals_request)

#----------------- start move action ----------------------
#print(response_goals)
rospy.loginfo('Executing state move')
try:
	rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
	move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
	move_request_object = my_goalRequest() # Create an object of type EmptyRequest
except rospy.ServiceException, e:
	print ("Service call move_robot_to_goal failed: %s"%e)

rospy.Timer(rospy.Duration(10), check_stop)#Check every 10s if time limit reached or stop message received
for i in range(len(response_goals.goals_to_reachx)):   
	#This begin_time is used to ensure that after a delay the action is stopped
	begin_time = (rospy.Time.from_sec(time.time())).to_sec()
	
	print("next goal:",response_goals.goals_to_reachx[i],response_goals.goals_to_reachy[i])
	#check odom
	#call_pose()  
	poseX,poseY = odom.getPose()
	#send goal to move action      
	move_request_object.x_goal=response_goals.goals_to_reachx[i]
	move_request_object.y_goal=response_goals.goals_to_reachy[i]
	#get the actual pose of the robot
	initialDistance = math.sqrt((response_goals.goals_to_reachx[i] - poseX)**2 + (response_goals.goals_to_reachy[i] - poseY)**2)
	initial_x = poseX
	initial_y = poseY
	print("actual pose and distance to goal:", initial_x, initial_y, initialDistance)
	#receive state success of the action
	response_move = move_to_goal(move_request_object)

	#If robot start to move toward goal
	if response_move.success==True:
		print("moved successfully")
		poseX,poseY = odom.getPose()
		while ((poseX > response_goals.goals_to_reachx[i] + POS_TOLERANCE or poseX < response_goals.goals_to_reachx[i] - POS_TOLERANCE) \
			or (poseY > response_goals.goals_to_reachy[i] + POS_TOLERANCE or poseY < response_goals.goals_to_reachy[i] - POS_TOLERANCE)) \
			and not math.sqrt((poseX - initial_x)**2 + (poseY - initial_y)**2) > initialDistance :
			poseX,poseY = odom.getPose()
			
			
			"""
			in check_stop:		
			if speech topic receive stop than stop robot and exit with a return stop
			check if execution too long, if yes stop robot/ for that recuperate the code i wrote in search map package
			"""
			
			if STOP== True:
				poseX,poseY = odom.getPose()
				print("actual pose", poseX, poseY)
				move_request_object.x_goal=poseX
				move_request_object.y_goal=poseY
				response_move = move_to_goal(move_request_object)
				STOP=False
				rospy.sleep(2)
				break
	else:
		print("error")
		#break

"""
continueT = False
pub.publish(continueT)
rospy.sleep(60)
continueT= True
pub.publish(continueT)
"""
