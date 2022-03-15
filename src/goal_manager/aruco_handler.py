#!/usr/bin/env python

import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_srvs.srv import SetBool

class ArucoHandler:
    def __init__(self):
        rospy.wait_for_service('/aruco_detect/enable_detections')
        self.mode = 0  # 0: nothing, 1: post mode, 2: gate mode
        self.enable_detections_service = rospy.ServiceProxy('/aruco_detect/enable_detections', SetBool)
        self.enable(False)
        rospy.loginfo("Starting Aruco Handler")
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_transforms_cb)

    def set_mode(self, mode):
        if 1 <= mode <= 2:
            self.mode = mode
        else:
            rospy.logerr("Invalid mode")

    def get_mode(self):
        return self.mode

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
