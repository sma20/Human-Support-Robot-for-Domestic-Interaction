#!/usr/bin/env python
#NOTE: 
#MAPPING- The map behing registered is called test1 and the mapping stops after 1 min, to change and take this map as reference for next uses
#There is a time constraint IN Mapping and IN the state machine

import roslib
import rospy, time
import roslaunch
import smach
import smach_ros
import math as math
import csv
import test
import thread
from threading import Thread

from architecture.srv import *
from geometry_msgs.msg import Twist, Point,  PoseStamped,  Quaternion
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#roslaunch mapping path
path_mapping="/home/cata/hsr_ws2/src/find_frontier/launch/find_front.launch"
#roslaunch search_map path
path_search_map="/home/cata/hsr_ws2/src/architecture/launch/search_map.launch"
#path to the csv file containing objects and objects pose
data_file="/home/cata/hsr_ws2/src/semantic_hsr/data/semantic_map.csv"

#--time check global var--
begin_time=0
max_duration=60*17 #max duration before considering something went wrong: 17min
STOP=False

#--to check the execution of a launch file (search_map/mapping)
job_done=False

#--to retrieve action and such from the speech recognition callback
name_object= "x"
action= "x"
place= "x"

#--to retrieve object pose from search map launch
object_pose = Point() 
#------------------- TIME CHECK FCT ------------------------

#Check the whole process time hasn't extended the maximum duration allowed

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
#----- Callback of chatter1, if there is a need to stop the processes
def callback_stop(finish):
    global STOP
    if 'stop' in finish.data:
        STOP = True
        print(finish.data)
    
	

#----- callback of search_map class (to receive information if the job is been finished or not)
def callback_job_done(finished):
    global job_done
    job_done= finished
	
#----- callback of chatter, there to extract the action, object and room (if relevant)
def callback_action(data):
    global name_object, action, place
    #execute actions
    list = data.split(',')
    name_object = list[0] 
    action = list[1] 
    place = list[2] 

def callback_objectpose(data):
    global object_pose
    object_pose.x=data.x
    object_pose.y=data.y
    object_pose.z=data.z


#----------------------- ACTIONS machine class ----------------------------------

#space for improvement, could replace the string by functions here (maybe to check if that is the wanted action)
def switch_actions(action_choice):
    switcher={
	    #it will send the numbers in a string now.
            1:'mapping',
            2:'get',
            3:'welcome', 
            #3:'find',
            #actions names
    }
    result=switcher.get(action,"nothing")
    return result
    


#Here we choose which action will be executed. maybe add a verification check with the user here
class choose_actions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['get','mapping','welcome','nothing']) # 'follow','other actions names])

    def execute(self,userdata):
        global action
        action=2
	
        #sunbul speech python script calls here
        #thread1 = Thread(target=thread.callback,args=())
        #thread1.start()
	thread.execThread()
        rospy.Subscriber("chatter", String, callback_action)#get as a string the action, object, room (if pertinent)



        #int(action)
        choice=switch_actions(action)
        return choice



#This function is there to avoid repeating twice the same action once the action finished.
class reset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rebooted']) #we could add a clean stop of the hsr here

    def execute(self,userdata):
        global name_object,action,place
        name_object= "x"
        action= "x"
        place= "x"
        return 'rebooted'

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

class retrieve_position_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_found','goal_not_found'],
                                    output_keys=['real_goal_position'])
    def execute(self,userdata):
        global data_file
        global name_object, place

        real_goal_position=Point()

        M=[]
        with open(data_file) as csvfile:
                reader = csv.reader(csvfile) # change contents to floats
                for row in reader: # each row is a list
                    M.append(row)
        csvfile.close
        k=0	

        #Brieuc you didn't test your code... you can't hide it
        for i in range(1,len(M)):
            if(name_object==M[i][0] and place==M[i][4]):
                real_goal_position.x=M[i][1]
                real_goal_position.y=M[i][2]
                real_goal_position.z=M[i][3]
                k=1
                break
        if(k==0):
            print("Object unknown")
            return 'goal_not_found'
        else:
            userdata.real_goal_position = real_goal_position
            return 'goal_found'


#call a launch to start this service. code too long, i wanted this py to be reserved for calling fct. to gives lisibility
#SEARCH_MAP.launch /search_map.py and map_exploration_service_v2.py -
#Search goals in the room/home and then send it to the move action.
#Brieuc read "search_map.py" , there is something for you there (retrieve room info + csv file search or yolo check)

#input: nonne
#output:nonne 
class search_map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'], output_keys=['real_goal_position'])#,input_keys=['name_object']) #For the upgraded version
    
    
    def execute(self,userdata):
        global path_search_map
        global begin_time
        global name_object, place
        global STOP
        global job_done
        global object_pose

        rospy.Timer(rospy.Duration(10), check_stop)#Check every 10s if time limit reached or stop message received
        #the roslaunch has been tested in test_launch, test again when we have the objects file
        sub = rospy.Subscriber('job_done', Bool, callback_job_done)
        sub2 = rospy.Subscriber('object_pose',Point, callback_objectpose)
        pub2 = rospy.Publisher('/room_object',String,queue_size=2) #we will send room and object to other node
        object_room= name_object +","+ place
        #we launch a full roslaunch here to start this stuff
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path_search_map])  
        launch.start()

        pub2.publish(object_room)
        rospy.loginfo("started")
        begin_time = (rospy.Time.from_sec(time.time())).to_sec()

        #while delay not expired or the search isn't finished
        while STOP==False and job_done==False:
            i=0
        #rospy.sleep(60*5)
        # 5min later
        rospy.loginfo("CLOOOOSE SEARCH_MAP LAUNCH")
        launch.shutdown()
        job_done=False #we reset job_done for next use
        STOP=False #We RESET stop for next time, in case STOP=True
        real_goal_position = Point()
        #If the object was found in search map

        if object_pose.z!=9000 :
            real_goal_position.x = object_pose.x
            real_goal_position.y = object_pose.y
            real_goal_position.z = object_pose.z
            userdata.real_goal_position = real_goal_position
            print("proceed to search point to reach")
            return 'success'

        userdata.real_goal_position = real_goal_position
        print("failure of the operation")
        return 'failure'	


      

#----------------------------- END SUB_GET CLASS ------------------------------------------

#---------------------------- SUB_WELCOME State machine classes -----------------------------------------------------------------
#The move service with a static point (always the same goal)

# MOVE SERVICE (move.py) - It moves the base of the robot with obstacle avoidance as an action (once on, state= success)
#parameters to enter: NONE DOOR POSITION FIXED in this execute
#output: none
class go_main_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','not_reached'])
            
    def execute(self,userdata):
        door_posex=0.75
        door_posey=-1 #Calculated by hand and specific to HW LAB. end corridor middle coordinates. Y -60cm to avoid the door opening on the robot
        rospy.loginfo('Executing state move')
        #call service
        try:
            rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
            move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
            move_request_object = my_goalRequest() # Create an object of type EmptyRequest
        except rospy.ServiceException, e:
            print ("Service call move_robot_to_goal failed: %s"%e)

        #x,y positions sent to move
        move_request_object.x_goal=door_posex
        move_request_object.y_goal=door_posey

        response_move = move_to_goal(move_request_object)

        if response_move.success==True:
            print("moved successfully")
            return 'success'
        else:
            return 'fail'

        #Here launch the move action with "if not succeded gives another mid_goal and retry"
        return 'reached'


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

#explain the fct here
#Name of the python files/ launch files used here
#Inputs:
#Outputs:
class check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['person_recognized','person_not_recognized'])
    
    
    def execute(self,userdata):

        return 'person_recognized'
#----------------------------- END SUB_WELCOME CLASS ------------------------------------------


#---------------------------- SUB_ACCOMPANY State machine classes -----------------------------------------------------------------

#----------------------------- END SUB_ACCOMPANY CLASS ------------------------------------------

#----------------------------- SUB_MAP State machine Class --------------------------------------------
#here the map is created, the robot moves autonomously in the room, map it with hector slam and save it in architecture/map_tests
#FIND_FRONT.launch is launched here it's in find_frontier package. 
#No inputs
#No outputs
class mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
    
    def execute(self,userdata):
        global path_search_map
        global begin_time
        global STOP
        global job_done

        rospy.Timer(rospy.Duration(10), check_stop)#Check every 10s if time limit reached or stop message received
        #the roslaunch has been tested in test_launch, test again when we have the objects file
        

        #we launch a full roslaunch here to start this stuff
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path_mapping])  
        launch.start()
        rospy.loginfo("started")
        begin_time = (rospy.Time.from_sec(time.time())).to_sec()

        #while delay not expired or the search isn't finished
        while STOP==False and job_done==False:
            i=0
        #rospy.sleep(60*5)
        # 5min later
        job_done=False
        STOP=False #We reset stop for next time, in case STOP=True
        rospy.loginfo("CLOOOOSE MAPING LAUNCH")
        launch.shutdown()
        return 'success'
	#to adapt to the real result

#------------------------------ END SUB_MAP State Machine class ---------------------------

# ----------------------------- Main (state machines) -------------------------------------
def main():
    rospy.init_node('smach_example_state_machine')
    print("first_smach_on")
    sub = rospy.Subscriber('job_done', Bool, callback_job_done)
    #rospy.Subscriber("chatter", String, callback_action) #get as a string the action, object, room (if pertinent)
    rospy.Subscriber("chatter1", String, callback_stop) #receive the stop or ultra stop here to stop what is ongoing or everything.

    #action state machine
    sm_actions = smach.StateMachine(outcomes=['finished'])


#-----------------------------actions STATE MACHINE -----------------------------
    #open container actions
    with sm_actions:

        smach.StateMachine.add('choose_actions', choose_actions(), 
                                transitions={'get':'GET','mapping':'MAPPING','welcome':'WELCOME','nothing':'RESET'}) #accompany  find
        smach.StateMachine.add('RESET', reset(), 
                                transitions={'rebooted':'choose_actions'}) #if you want to ask if hsr has to sleep. here you can
        # Create a GET state machine
        sm_get = smach.StateMachine(outcomes=['get_success', 'get_failure'])
        
        sm_get.userdata.real_goal_position = Point()
        sm_get.userdata.position_goal= Point()

        #for testing GET //WILL HAVE to be automatized:
        #sm_get.userdata.real_goal_position.x=2
        #sm_get.userdata.real_goal_position.y=6

        #Create a MAPPING state machine
        sm_map = smach.StateMachine(outcomes=['map_success', 'map_failure'])

        #Create a WELCOME state machine (check who is at the front door)
        sm_welcome = smach.StateMachine(outcomes=['welcome_success', 'welcome_failure'])


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
                                    'failure':'get_failure'},  
				   remapping={'real_goal_position':'real_goal_position'})


#----------------------------- END GET State Machine ------------------------------
#----------------------------- MAPPING State Machine ------------------------------
        with sm_map:
            #Search space to find the object
            smach.StateMachine.add('mapping', mapping(),transitions={'success':'map_success',
                                'failure':'map_failure'})

#----------------------------- END MAPPING State Machine ---------------------------

#---------------------------- WELCOME State Machine ---------------------------------

# Or if you don't succeed tell me your combined strategy if you want me to add it here.

        with sm_welcome:
            #go at the door 
            smach.StateMachine.add('go_main_door',go_main_door(),transitions={'reached':'check_person',
                                'not_reached':'go_main_door'})#Not a good idea to create a loop here, to change later on
	        #move action
            smach.StateMachine.add('move', move(), 
                            transitions={'success':'welcome_success', #TO CHANGE. WAITING FOR YOUR CODES TO FILL THIS ARCHITECTURE
                                        'fail':'welcome_failure'},
                            remapping={'position_goal':'position_goal'})
            #check who is there
            #Brieuc, maybe adding the front door position in the CSV file would be usefull for going there
            
            smach.StateMachine.add('check_person', check(),transitions={'person_recognized':'welcome_success',
                                'person_not_recognized':'welcome_failure'}) #NOTE: welcome success and failure will have to be replace with the name of the next fct you want to call in State machine

            #ADD YOUR STATE MACHINE FCTS



#---------------------------- END WELCOME State Machine -----------------------------

        #link between sm_action and sm_get. sm_get called by sm_actions
        smach.StateMachine.add('GET', sm_get, 
                                transitions={'get_success':'RESET', 'get_failure':'RESET'})
        #link between sm_action and sm_map. sm_map called by sm_actions
        smach.StateMachine.add('MAPPING', sm_map, 
                                transitions={'map_success':'RESET', 'map_failure':'RESET'})
        #link between sm_action and sm_welcome called by sm_actions
        smach.StateMachine.add('WELCOME', sm_welcome, 
                                transitions={'welcome_success':'RESET', 'welcome_failure':'RESET'})
        
        
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
    rospy.spin()
