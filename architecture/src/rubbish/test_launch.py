#!/usr/bin/env python

import roslaunch
import rospy
from std_msgs.msg import Bool


Delay=False

def callback(msg):
    global Delay
    Delay= msg  


rospy.init_node('search_map', anonymous=True)
rospy.Subscriber("fake_pub", Bool, callback)


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/cata/hsr_ws2/src/search_map/launch/search_map.launch"])
launch.start()
rospy.loginfo("started")

while Delay==False:
    i=0
#rospy.sleep(60*5)
# 5min later
rospy.loginfo("CLOOOOOSE LAUNCH")
launch.shutdown()





