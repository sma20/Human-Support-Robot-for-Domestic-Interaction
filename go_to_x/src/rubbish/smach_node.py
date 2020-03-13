#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from go_to_x.srv import *
from geometry_msgs.msg import Twist, Point,  PoseStamped,  Quaternion
# define state Foo



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
# outputs: a position x,y,z
class search_closest_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['position_found'],input_keys=['real_goal_position'],
                                    output_keys=['position_goal'])
        self.retry=0
        self.prev_real_goal_position = Point()
        self.start_set_goal=False

    def execute(self, userdata):
        rospy.loginfo('Executing search closest accessible point')
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
        



# main
def main():
    rospy.init_node('smach_example_state_machine')
    print("fist_smach_on")
 

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['get_success', 'get_failure'])
    sm.userdata.real_goal_position = Point()
    sm.userdata.position_goal= Point()

    #for testing:
    sm.userdata.real_goal_position.x=2
    sm.userdata.real_goal_position.y=6
    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('search_closest_position', search_closest_position(), 
                               transitions={'position_found':'move'},
                                remapping={'real_goal_position':'real_goal_position', 
                                             'position_goal':'position_goal'})
        smach.StateMachine.add('move', move(), 
                        transitions={'success':'get_success', 
                                    'fail':'get_failure'},
                        remapping={'position_goal':'position_goal'})

    sis = smach_ros.IntrospectionServer('test_state_machine', sm, '/SECOND_SM')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
