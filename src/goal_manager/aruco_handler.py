#!/usr/bin/env python

import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_srvs.srv import SetBool

class ArucoHandler:
    def __init__(self):
        self.post_ids = [1, 2, 3]  # TODO: put correct ids
        self.gate_ids = [[4, 5], [6, 7]]  # TODO: put correct ids. Does a gate have two ids or just one?
        rospy.wait_for_service('/aruco_detect/enable_detections')
        self.enable_detections_service = rospy.ServiceProxy('/aruco_detect/enable_detections', SetBool)
        self.enable(False)
        rospy.loginfo("Starting Aruco Handler")
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_transforms_cb)

    def enable(self, state):
        try:
            resp = self.enable_detections_service(state)
            if state:
                rospy.loginfo("Enabling detection")
            else:
                rospy.loginfo("Disabling detection")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def fiducial_transforms_cb(self, msg):
        for transform in msg.transforms:
            print("Found id=%d" % transform.fiducial_id)
            if transform.fiducial_id in self.post_ids:
                rospy.loginfo("Post found")
            elif transform.fiducial_id in self.gate_ids:
                rospy.loginfo("Gate found")
