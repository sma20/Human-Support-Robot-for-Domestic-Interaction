#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool


pub = rospy.Publisher('fake_pub', Bool, queue_size=1)
rospy.init_node('fake_pub', anonymous=True)

while not rospy.is_shutdown():
    continueT = False
    pub.publish(continueT)
    rospy.sleep(60)
    continueT= True
    pub.publish(continueT)

