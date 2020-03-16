#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist #to send the command
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def callback(msg):
   
    goal=PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "odom"
    goal.pose.position = Point(msg.x, msg.y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, msg.z)
    goal.pose.orientation = Quaternion(*quat)

    # publish ROS message
    pub.publish_once(goal)



def publish_in_cmd(goal):
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems, this is no big deal, but in systems that publish only
    once, it IS very important.
    """
    pub= rospy.Publisher("/goal",PoseStamped,queue_size=4)
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(goal)

            #rospy.loginfo("Cmd Published")
        else:
            rospy.sleep(0.1)

if __name__ == '__main__':

    rospy.init_node('go') # Initialise THE ROS node
    print("go on")

    # initialize ROS publisher
    pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
    sub= rospy.Subscriber("position_goal", Point, callback)#TO SET in the smach from set_goal service to smach and smach to here as topic
    
