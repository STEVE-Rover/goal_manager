#!/usr/bin/env python

import rospy
from math import sqrt
from fiducial_msgs.msg import FiducialTransformArray
from std_srvs.srv import SetBool

class ArucoHandler:
    def __init__(self):
        rospy.wait_for_service('/aruco_detect/enable_detections')
        self.mode = 0  # 0: nothing, 1: post mode, 2: gate mode
        self.duplicate_dist_thresh = rospy.get_param("~duplicate_dist_thresh", 0.25)
        self.enable_detections_service = rospy.ServiceProxy('/aruco_detect/enable_detections', SetBool)
        self.enable(False)
        rospy.loginfo("Starting Aruco Handler")
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducials_cb)

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

    def fiducials_cb(self, msg):
        if len(msg.transforms) == 0 or not self.is_fiducials_valid(msg):
            return
        msg.transforms = self.remove_duplicates(msg.transforms)
        if self.mode == 1:  # Post
            if len(msg.transforms) != 1:
                return
            else:
                # TODO: commit
                pass

        elif self.mode == 2:  # Gate
            if len(msg.transform) != 2:
                return
            else:
                # TODO: commit
                pass
        
    def is_fiducials_valid(self, fiducials):
        ids = []
        id_indexes = []
        for i in range(len(fiducials.transforms)):
            try:
                index = ids.index(fiducials.transforms[i].fiducial_id)
                # Duplicate ID found. If the distance between the two is small, that means the duplicate is
                # just the another of the three faces of the visual marker. If the distance is big, that means
                # there is a problem with the detection.
                if calc_dist(fiducials.transforms[i].transform.translation, fiducials.transforms[id_indexes[index]].translation) >= self.duplicate_dist_thresh:
                    return False
            except ValueError:
                pass
                
            ids.append(fiducials.transforms[i].fiducial_id)
            id_indexes.append(i)
        return True

    def remove_duplicates(self, fiducials):
        ids = []
        filtered_fiducials = []
        for fiducial in fiducials:
            if fiducial.fiducial_id in ids:
                filtered_fiducials.append(fiducial)
            ids.append(fiducial.fiducial_id)
        return filtered_fiducials


def calc_dist(position1, position2):
    return sqrt((position2.x - position1.x)**2 + 
                (position2.y - position1.y)**2 +
                (position2.z - position1.z)**2)
