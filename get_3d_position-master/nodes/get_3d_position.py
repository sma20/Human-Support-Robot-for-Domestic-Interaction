#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PointStamped
from tmc_geometry_msgs.msg import Point2DStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform,
                                             InversePerspectiveTransformRequest)
from tmc_darknet_msgs.msg import Detections

_CONNECTION_TIMEOUT = 10.0
_SENSOR_FRAME_ID = 'head_rgbd_sensor_rgb_frame'

class Get3DPosition(object):

    def __init__(self):
        self._visualize = rospy.get_param('~debug', False)
        self._client = rospy.ServiceProxy('/inverse_perspective_transform',
                                          InversePerspectiveTransform)
        self._obj_detection_sub = rospy.Subscriber(
            '/yolo2_node/detections', Detections, self._obj_detection_cb)
        if self._visualize:
            self._result_pub = rospy.Publisher(
                '/3d_position', PointStamped, queue_size=10)

    def _obj_detection_cb(self, msg):
        try:
            self._client.wait_for_service(timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            return
        # Get the position of the target object from a camera image
        req = InversePerspectiveTransformRequest()
        req.target_frame = _SENSOR_FRAME_ID
        req.depth_registration = True
        for detection in msg.detections:
            target_point = Point2DStamped()
            target_point.point.x = int(detection.x)
            target_point.point.y = int(detection.y)
            req.points_2D.append(target_point)
        try:
            res = self._client(req)
            rospy.loginfo(res)
            if self._visualize:
                for point in res.points_3D:
                    point_3d = PointStamped()
                    point_3d.header = point.header
                    point_3d.point =  point.point
                    self._result_pub.publish(point_3d)
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
