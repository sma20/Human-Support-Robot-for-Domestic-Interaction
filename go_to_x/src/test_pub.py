#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool
from go_to_x.srv import find_goals,find_goalsRequest, find_goalsResponse

#pub = rospy.Publisher('fake_pub', Bool, queue_size=1)
rospy.init_node('fake_pub', anonymous=True)
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

print(response_goals)

"""
continueT = False
pub.publish(continueT)
rospy.sleep(60)
continueT= True
pub.publish(continueT)
"""
