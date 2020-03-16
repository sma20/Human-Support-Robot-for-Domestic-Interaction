#! /usr/bin/env python
#NOTE: if using an action to move REAL ROBOT. need to do "initialization self-position of the robot"
import rospy
import math as math
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist 
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler#
from go_to_x.srv import my_goal, my_goalResponse

poseX= 0.0
poseY= 0.0
POS_TOLERANCE = 0.02

#since this /goal topic already has obstacle avoidance included, we can safely remove the laser scan check
def my_callback(request):
    rospy.loginfo("move_robot Service called")
    global poseX
    global poseY
    goal_yaw=0
    i=0
    response=my_goalResponse()

    #rospy.loginfo("move_robot Service called")
    call_pose()
    print("x_goal,y_goal")
    print(request.x_goal,request.y_goal)

    #warning: set closest goal see the axis of the map differently than the map here
    #so x and y are to be interverted
    response.success=new_hsr_function(request.x_goal,request.y_goal)


    #rospy.loginfo("Finished move_robot service")
    return response



def call_pose():
    """to call the odometry"""
    sub = rospy.Subscriber('/hsrb/odom', Odometry, getPose)

def getPose(msg):
    """to get all the pose we need as global, could be done with a return, but well"""
    global poseX
    global poseY

    poseX,poseY=msg.pose.pose.position.x,msg.pose.pose.position.y
    

def publish_in_cmd(goal_begin):
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems, this is no big deal, but in systems that publish only
    once, it IS very important.
    """
    pub= rospy.Publisher("goal",PoseStamped,queue_size=10)
    #rospy.loginfo("here")
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)
    #print ("Connected?")
    pub.publish(goal_begin)
    rospy.sleep(5)
    rospy.loginfo("Cmd Published")



def new_hsr_function(destination_x,destination_y):
    global POS_TOLERANCE
    global poseX
    global poseY

    # initialize action client
    cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    # wait for the action server to establish connection
    cli.wait_for_server()


    goal_begin=PoseStamped()
    
    initialDistance = math.sqrt((destination_x - poseX)**2 + (destination_y - poseY)**2)
    initial_x = poseX
    initial_y = poseY

    rate = rospy.Rate(2)

    destination_angle = math.atan2(destination_y - poseY, destination_x - poseX)


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

    # print result of navigation
    action_state = cli.get_state()
    if action_state == GoalStatus.SUCCEEDED:
        goodTermination=True
        print("goal reached")
    

    return goodTermination



rospy.init_node('move_to_goal')
my_service = rospy.Service('/move_robot_to_goal', my_goal , my_callback)
rospy.loginfo("MOVE_ROBOT Service Ready")

rospy.spin()
