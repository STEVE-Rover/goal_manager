#!/usr/bin/env python

import rospy
from math import sqrt
from fiducial_msgs.msg import FiducialTransformArray
from std_srvs.srv import SetBool

#TODO: ArucoHandler should disable itself when the goal is reached
class ArucoHandler:
    def __init__(self, parent):
        self.parent = parent
        rospy.loginfo("Waiting for aruco_detect")
        rospy.wait_for_service('/aruco_detect/enable_detections')
        self.mode = 0  # 0: nothing, 1: post mode, 2: gate mode
        self.enabled = False
        self.goal_in_progess = False
        self.duplicate_dist_thresh = rospy.get_param("~duplicate_dist_thresh", 0.25)
        self.enable_detections_service = rospy.ServiceProxy('/aruco_detect/enable_detections', SetBool)
        self.enable(False)
        rospy.loginfo("Starting Aruco Handler")
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducials_cb)

    def set_mode(self, mode):
        if 0 <= mode <= 2:
            self.mode = mode
        else:
            rospy.logerr("Invalid mode")

    def enable(self, state):
        try:
            self.enabled = state
            resp = self.enable_detections_service(state)
            if state:
                rospy.loginfo("Enabling detection")
            else:
                self.goal_in_progess = False
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
            # TODO: recommit if the distance between the current tag and the previous detection is significant
            elif not self.goal_in_progess:
                self.commit_post(msg.transforms[0].transform)

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
            if fiducial.fiducial_id not in ids:
                filtered_fiducials.append(fiducial)
            ids.append(fiducial.fiducial_id)
        return filtered_fiducials


    def commit_post(self, transform):
        rospy.loginfo("Post found! Creating goal.")
        self.goal_in_progess = True
        # The transform is in the camera frame, with the Z axis pointing out of the lens
        transform.translation.z -= 1  # Offset the goal 1m in front of the post
        # Ignore the fiducial's orientation, we just want the rover to reach the position
        # TODO: make sure this orientation is valid for move_base
        transform.rotation.x = 0
        transform.rotation.y = 0
        transform.rotation.z = 0
        transform.rotation.w = 1
        # TODO: transform into the map frame before publishing the goal
        self.parent.publish_goal_quat(transform.translation.x, transform.translation.y, transform.translation.z,
                                      transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)

    def commit_gate(self, transform1, transform2):
        pass
        #TODO: complete this function


def calc_dist(position1, position2):
    return sqrt((position2.x - position1.x)**2 + 
                (position2.y - position1.y)**2 +
                (position2.z - position1.z)**2)
