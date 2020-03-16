#!/usr/bin/env python

import roslib
import rospy, time
import roslaunch
import smach
import smach_ros
import math as math

from go_to_x.srv import *
from geometry_msgs.msg import Twist, Point,  PoseStamped,  Quaternion
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler#


#roslaunch search_map path
path_search_map="/home/cata/hsr_ws2/src/go_to_x/launch/search_map.launch"

#--time check global var--
begin_time=0
max_duration=60*10 #max duration before considering something went wrong: 10min
STOP=False

#--to check the execution of a launch file (search_map)
job_done=False

#------------------- TIME CHECK FCT ------------------------

#Check the whole process time hasn't extended the maximum duration allowed
#Sunbul: would be great to read the topic you created here to check if a "STOP" command was received to stop everything directly here
#Daria: i did not implemented the stop command yet. its a tricky one. might have to make it a service and i dont know how to make google API a service.:D
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
		#fct to use for long classes actions
		#rospy.is_shutdown()
		#to decide what to do here
		"""
		else if speech topic receive stop than stop robot and exit with a return stop
		"""
		#print 'Timer called at ' + str(event.current_real)



#----------------------- SUBSCRIBER CALLBACKS ----------------------------------
#----- callback of search_map class (to receive information if the job is been finished or not)

def callback_job_done(finished):
    global job_done
    job_done= finished

#sunbul: if you wish to add yur callback here




#----------------------- ACTIONS machine class ----------------------------------

#space for improvement, could replace the string by functions here (maybe to check if that is the wanted action)
def switch_actions(action_choice):
    switcher={
            1:'get',
            #2:'accompany', 
            #3:'find',
            #actions names
    }
    result=switcher.get(action_choice,"nothing")
    return result
    


#Here we choose which action will be executed. maybe add a verification check with the user here
class choose_actions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['get','nothing']) # 'follow','other actions names])

    def execute(self,userdata):
        action_choice=1
        #Sunbul: your action info is to be retrieve from the topic here. only the action/the whole topic if you wish (then it will have to be passed as an output to the next state machine)
        
        choice=switch_actions(action_choice)
        return choice


#---------------------------- SUB_GET State machine classes -----------------------------------------------------------------

#Possible upgrade for move: Move service actually send a goal to a topic /goal. 
#It could be possible to call it directly from the state machine see: http://library.isr.ist.utl.pt/docs/roswiki/smach(2f)Tutorials(2f)Calling(20)Actions.html

# MOVE SERVICE (move.py) - It moves the base of the robot with obstacle avoidance as an action (once on, state= success)
#parameters to enter: position x, y as a point 
#output: none

#NOTE: want to change move by move_action. so if path not ok, we know it thanks to action status.
class move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'], input_keys=['position_goal'])

    def execute(self, userdata):
        rospy.loginfo('Executing state move')
        #call service
        try:
            rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
            move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
            move_request_object = my_goalRequest() # Create an object of type EmptyRequest
        except rospy.ServiceException, e:
            print ("Service call move_robot_to_goal failed: %s"%e)

        #x,y positions sent to move
        move_request_object.x_goal=userdata.position_goal.x
        move_request_object.y_goal=userdata.position_goal.y

        response_move = move_to_goal(move_request_object)

        if response_move.success==True:
            print("moved successfully")
            return 'success'
        else:
            return 'fail'


# FIND CLOSEST GOAL SERVICE (set_goal_v3.py) - It finds the closest accessible point around the goal and output it
# parameters to enter: position x,y,z of the goal (object/person)as a point
# outputs: a position x,y,z as a point
class search_closest_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['position_found'],input_keys=['real_goal_position'],
                                    output_keys=['position_goal'])
        self.retry=0
        self.prev_real_goal_position = Point()
        self.start_set_goal=False

    def execute(self, userdata):
        rospy.loginfo('Executing search closest accessible point, service on?')
        #call service
        try:
            rospy.wait_for_service('/set_goal')# Wait for the service to be running (with launch file)
            set_goal_service = rospy.ServiceProxy('/set_goal', set_goal) # Create connection to service
            request_object = set_goalRequest() # Create an object of type EmptyRequest
        except rospy.ServiceException, e:
            print ("Service call set_closest_goal failed: %s"%e)
        #if we send twice the same goal, it means we are retrying to get to it
        if self.prev_real_goal_position != userdata.real_goal_position:
            self.retry=0 #if we search position for a new goal retry=0
        print("service set_closest point on")
        start_set_goal=True

        request_object.start_set_goal=start_set_goal
        request_object.real_goal_position=userdata.real_goal_position
        request_object.retry=self.retry

        response_set_goal = set_goal_service(request_object)
        print("received response")
        if response_set_goal.success==False: 
            print("can't find goal position, we have to implement a solution")

        #if we received a goal
        start_set_goal=False
        self.retry+=1
        print("finished searching")
        #send back goal
        userdata.position_goal=response_set_goal.position_goal

        return 'position_found'


# To create: a function that search in a file an object name and retrieve its position as a point       
# parameters to enter: object, room (or home if any)
# outputs: a position x,y,z as a point
#Brieuc: you became the csv expert now ;) 
#Sunul: extraction of the object name here

class retrieve_position_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_found','goal_not_found'],
                                    output_keys=['real_goal_position'])
    def execute(self,userdata):
    	return 'goal_not_found'



#call a launch to start this service. code too long, i wanted this py to be reserved for calling fct. to gives lisibility
#SEARCH_MAP.launch /search_map.py and map_exploration_service_v2.py -
#Search goals in the room/home and then send it to the move action.
#Brieuc, Sunbul: read "search_map.py" , there is something for you there (retrieve room info + csv file search or yolo check)

#input: nonne
#output:nonne 
class search_map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])#,input_keys=['name_object']) #For the upgraded version
    
    
    def execute(self,userdata):
        global path_search_map
        global begin_time

        rospy.Timer(rospy.Duration(10), check_stop)#Check every 10s if time limit reached or stop message received
        #the roslaunch has been tested in test_launch, test again when we have the objects file
        sub = rospy.Subscriber('job_done', Bool, callback_job_done)

        #we launch a full roslaunch here to start this stuff
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path_search_map])  
        launch.start()
        rospy.loginfo("started")
        begin_time = (rospy.Time.from_sec(time.time())).to_sec()

        #while delay not expired or the search isn't finished
        while STOP==False or job_done==False:
            i=0
        #rospy.sleep(60*5)
        # 5min later
        rospy.loginfo("CLOOOOSE SEARCH_MAP LAUNCH")
        launch.shutdown()
        return 'success'#to adapt to the real result
        
        

#----------------------------- END SUB_GET CLASS ------------------------------------------

#---------------------------- SUB_FIND State machine classes -----------------------------------------------------------------


#----------------------------- END SUB_FIND CLASS ------------------------------------------


#---------------------------- SUB_ACCOMPANY State machine classes -----------------------------------------------------------------

#----------------------------- END SUB_ACCOMPANY CLASS ------------------------------------------



# ----------------------------- Main (state machines) -------------------------------------
def main():
    rospy.init_node('smach_example_state_machine')
    print("first_smach_on")

    #action state machine
    sm_actions = smach.StateMachine(outcomes=['finished'])


#-----------------------------actions STATE MACHINE -----------------------------
    #open container actions
    with sm_actions:

        smach.StateMachine.add('choose_actions', choose_actions(), 
                                transitions={'get':'GET', 'nothing':'finished'}) #accompany  find


        # Create a GET state machine
        sm_get = smach.StateMachine(outcomes=['get_success', 'get_failure'])
        
        sm_get.userdata.real_goal_position = Point()
        sm_get.userdata.position_goal= Point()

        #for testing GET //WILL HAVE to be automatized:
        sm_get.userdata.real_goal_position.x=2
        sm_get.userdata.real_goal_position.y=6

#----------------------------- GET STATE MACHINE ----------------------------------
        # Open the container GET
        with sm_get:
            # Add states to the container

            #Search position of the object
            smach.StateMachine.add('retrieve_position_object', retrieve_position_object(),transitions={'goal_found':'search_closest_position',
                                    'goal_not_found':'search_map'},
                                    remapping={'real_goal_position':'real_goal_position'})
            
            #Search closest accessible point to the object
            smach.StateMachine.add('search_closest_position', search_closest_position(), 
                                    transitions={'position_found':'move'},
                                    remapping={'real_goal_position':'real_goal_position', 
                                                    'position_goal':'position_goal'})
            #Move toward object
            smach.StateMachine.add('move', move(), 
                            transitions={'success':'get_success', 
                                        'fail':'get_failure'},
                            remapping={'position_goal':'position_goal'})
            #Search space to find the object
            smach.StateMachine.add('search_map', search_map(),transitions={'success':'search_closest_position',
                                    'failure':'get_failure'})


#----------------------------- END GET State Machine ------------------------------

        #link between sm_action and sm_get. sm_get called by sm_actions
        smach.StateMachine.add('GET', sm_get, 
                                transitions={'get_success':'finished', 'get_failure':'finished'})
        
        
        #initialization sm_action for the visualizer "rosrun smach_viewer smach_viewer.py"
        sis = smach_ros.IntrospectionServer('test_state_machine', sm_actions, '/Action_SM')
        sis.start()

        # Execute the state machine
        outcome = sm_actions.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
