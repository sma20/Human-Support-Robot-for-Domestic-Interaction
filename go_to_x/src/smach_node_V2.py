#!/usr/bin/env python

import roslib
import rospy
import roslaunch
import smach
import smach_ros

from go_to_x.srv import *
from geometry_msgs.msg import Twist, Point,  PoseStamped,  Quaternion
from std_msgs.msg import Bool
# define state Foo



#----------------------- ACTIONS machine class ----------------------------------

#qpace for improvement, could replace the string by functions here (maybe to check if that is the wanted action)
def switch_actions(action_choice):
    switcher={
            1:'get',
            #2:'accompany', 
            #3:'find',
    }
    result=switcher.get(action_choice,"nothing")
    return result
    



#Here we choose which action will be executed. maybe add a verification check with the user here
class choose_actions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['get','nothing']) # 'follow','other actions names])

    def execute(self,userdata):
        action_choice=1
    # to determine if a state machine send in a number or if the informations are picked here (vocal info)
        
        choice=switch_actions(action_choice)
        return choice


#---------------------------- SUB_GET State machine classes -----------------------------------------------------------------


#Possible upgrade for move: Move service actually send a goal to an action /goal. 
#It could be possible to call it directly from the state machine see: http://library.isr.ist.utl.pt/docs/roswiki/smach(2f)Tutorials(2f)Calling(20)Actions.html

# MOVE SERVICE (move.py) - It moves the base of the robot with obstacle avoidance as an action (once on, state= success)
#parameters to enter: position x, y as a point 
#output: none

class move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'], input_keys=['position_goal'])

    def execute(self, userdata):
        rospy.loginfo('Executing state move')
        try:
            rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
            move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
            move_request_object = my_goalRequest() # Create an object of type EmptyRequest
        except rospy.ServiceException, e:
            print ("Service call move_robot_to_goal failed: %s"%e)
        move_request_object.x_goal=userdata.position_goal.x
        move_request_object.y_goal=userdata.position_goal.y

        response_move = move_to_goal(move_request_object)

        if response_move.success==True:
            print("moved successfully")
            return 'success'
        else:
            return 'fail'

# FIND CLOSEST GOAL SERVICE (set_goal_v3.py) - It find the closest accessible point around the goal and output it
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
        try:
            rospy.wait_for_service('/set_goal')# Wait for the service to be running (with launch file)
            set_goal_service = rospy.ServiceProxy('/set_goal', set_goal) # Create connection to service
            request_object = set_goalRequest() # Create an object of type EmptyRequest
        except rospy.ServiceException, e:
            print ("Service call set_closest_goal failed: %s"%e)
        if self.prev_real_goal_position != userdata.real_goal_position:
            self.retry=0 #if we search position for a new goal
        print("service set_closest point on")
        start_set_goal=True

        request_object.start_set_goal=start_set_goal
        request_object.real_goal_position=userdata.real_goal_position
        request_object.retry=self.retry

        response_set_goal = set_goal_service(request_object)
        print("received response")
        if response_set_goal.success==False: 
            print("can't find goal position, we have to implement a solution")

        #if we have a goal
        start_set_goal=False
        self.retry+=1
        print("finished searching")
        userdata.position_goal=response_set_goal.position_goal

        return 'position_found'


# To create: a function that search in a file an object name and retrieve its position as a point       
# parameters to enter: 
# outputs: a position x,y,z as a point

#TO DETERMINE: will we need to give the object name as this state is called? 
"""
class retrieve_position_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_found'],
                                    input_keys=['name_object']
                                    output_keys=['real_goal_position'])
    def execute(self,userdata):

"""

#To create: a function that search the map for the object
#For now using find_frontier with a time limit. 
#prob: not an action, risk of staying blocked in there for 10min (max time)
#Possible upgrade: to call an action/service with the object name, and travel the map with move in an ordonned way. 
#(repeat the action a number of time, after a number of try: STOP. If object found: END)


#tested in test_launch, test again when we have the objects file
"""
class search_map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])#,input_keys=['name_object']) #For the upgraded version
    
    
    def execute(self,userdata):
        rospy.init_node('search_map', anonymous=True)
        rospy.Subscriber("fake_pub", Bool, callback) #change it with the real topic


        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/cata/hsr_ws2/src/search_map/launch/search_map.launch"]) #change path 
        launch.start()
        rospy.loginfo("started")

        while Delay==False:
            i=0
        #rospy.sleep(60*5)
        # 5min later
        rospy.loginfo("CLOOOOOSE LAUNCH")
        launch.shutdown()

        

"""
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
            """
            smach.StateMachine.add('retrieve_position_object', retrieve_position_object(),transitions={'goal_found':'search_closest_position',
                                    'goal_not_found':'search_map'},
                                    remapping={'name_object':'name_object', 'real_goal_position':'real_goal_position'})
            """
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



#----------------------------- END GET State Machine ------------------------------


        smach.StateMachine.add('GET', sm_get, 
                                transitions={'get_success':'finished', 'get_failure':'finished'})
        
        
        
        sis = smach_ros.IntrospectionServer('test_state_machine', sm_actions, '/SECOND_SM')
        sis.start()

        # Execute the state machine
        outcome = sm_actions.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
