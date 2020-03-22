#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs #import the packages first
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PointStamped
from tmc_geometry_msgs.msg import Point2DStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform,
                                             InversePerspectiveTransformRequest)
from tmc_vision_msgs.msg import DetectionArray
#tmc_darknet_msgs.msg import Detections
_CONNECTION_TIMEOUT = 10.0
_SENSOR_FRAME_ID = 'head_rgbd_sensor_rgb_frame'




class Get3DPosition(object):

    def __init__(self):
        self._visualize = rospy.get_param('~debug', False)
        self._client = rospy.ServiceProxy('/inverse_perspective_transform',
                                          InversePerspectiveTransform)
	self._sub_odom = rospy.Subscriber("/global_pose", PoseStamped, self._find_position)
        self._obj_detection_sub = rospy.Subscriber(
            '/yolo2_node/detections', DetectionArray, self._obj_detection_cb)
	self._result_pub2 = rospy.Publisher(
                '/3d_position_label', PointStamped, queue_size=10)
	topic = 'visualization_marker_array'
	self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
	self._result_pub = rospy.Publisher(
                '/3d_position', PointStamped, queue_size=10)
        if self._visualize:
            self._result_pub = rospy.Publisher(
                '/3d_position', PointStamped, queue_size=10)
	    self._result_pub2 = rospy.Publisher(
                '/3d_position_label', PointStamped, queue_size=10)
    def _find_position(self, arg):
	global odom_x
	global odom_y
	global theta
	odom_x = round(arg.pose.position.x,2)
    	odom_y = round(arg.pose.position.y,2)
    	#print("x:",x)
    	#print("y:",y)


    	rot_q = arg.pose.orientation
    	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def _obj_detection_cb(self, msg):
        try:
            self._client.wait_for_service(timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            return
	global odom_x
	global odom_y
	global theta
        # Get the position of the target object from a camera image
        req = InversePerspectiveTransformRequest()
        req.target_frame = _SENSOR_FRAME_ID
        req.depth_registration = True
        i=0
	u=0
	table=[]
	
	
        for detection in msg.detections:
            target_point = Point2DStamped()
            target_point.point.x = int(detection.x)
            target_point.point.y = int(detection.y)
	    #target_point.header.frame_id= detection.label.name
	    table.append(detection.label.name)
	    print(detection.label.name)
            req.points_2D.append(target_point)

        try:
	    markerArray = MarkerArray()
            res = self._client(req)
	    for point in res.points_3D:
		test=PoseStamped()
		test.header=point.header
		test.header.frame_id='head_rgbd_sensor_rgb_frame'
		test.pose.position=point.point
		marker = Marker()
		marker.header.frame_id ="base_link" 
		marker.text= table[i]
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
   		marker.color.g = 1.0
   		marker.color.b = 0.0
		#marker.pose.orientation.w = 1.0
		
		point_3d = PointStamped()
                point_3d.header = point.header
                point_3d.point =  point.point
		self._result_pub.publish(point_3d)
		
		
		#rospy.loginfo(point_3d.point)
		#rospy.loginfo(marker)
		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		tf_listener = tf2_ros.TransformListener(tf_buffer)
		transform = tf_buffer.lookup_transform("base_link",
                                   test.header.frame_id, #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0))
		pose_transformed = tf2_geometry_msgs.do_transform_pose(test, transform)
		#print("pose_transformed",pose_transformed.pose)
		marker.pose = pose_transformed.pose
		markerArray.markers.append(marker)
		
		
	    	table_pos = PointStamped()
		table_pos.header=point.header
	    	table_pos.header.frame_id=table[i]
	    	table_pos.point= marker.pose.position
		#print(table_pos.point)
		i=i+1
		self._result_pub2.publish(table_pos)
		rospy.sleep(.01)
                #rospy.loginfo(table_pos)
	    
	    
		u=u+1
	    id = 0
	    
  	    for m in markerArray.markers:
            	m.id = id
            	id += 1

             # Publish the MarkerArray
            self.publisher.publish(markerArray)

   	    
	    print("get Position : Done")
            rospy.sleep(0.01)
            if self._visualize:
                for point in res.points_3D:
                    point_3d = PointStamped()
                    point_3d.header = point.header
                    point_3d.point =  point.point
		    table_pos = PointStamped()
		    table_pos.header=table[i]
		    table_pos.point= point.point
                    self._result_pub.publish(point_3d)
		    self._result_pub2.publish(table_pos)
		    #rospy.loginfo(table_pos)
		    i=i+1
                    rospy.sleep(.01)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return
        if len(res.points_3D) < 1:
            rospy.logerr('There is no detected point')
            return

def main():
    rospy.init_node('get_3d_position')
    node = Get3DPosition()
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)

if __name__ == '__main__':
    main()
