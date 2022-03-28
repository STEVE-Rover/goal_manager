#!/usr/bin/env python

import rospy
import tf
from math import sqrt, cos, pi
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

class ArucoHandler:
    def __init__(self, parent):
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("Waiting for aruco_detect")
        rospy.wait_for_service('/aruco_detect/enable_detections')
        self.parent = parent
        self.map_frame = "map"
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
                self.commit_post(msg.transforms[0].transform, msg.header)

        elif self.mode == 2:  # Gate
            if len(msg.transform) != 2:
                return
            else:
                self.commit_gate(msg.transform[0].transform, msg.transform[1].transform, msg.header)
        
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


    def commit_post(self, transform, header):
        rospy.loginfo("Post found! Creating goal.")
        self.goal_in_progess = True
        pose_cam_frame = transform_to_pose_stamped(transform, header)
        # The transform is in the camera frame, with the Z axis pointing out of the lens
        pose_cam_frame.pose.position.z -= 1  # Offset the goal 1m in front of the post
        # Ignore the fiducial's orientation, we just want the rover to reach the position
        pose_cam_frame.pose.orientation.x = 0
        pose_cam_frame.pose.orientation.y = 0
        pose_cam_frame.pose.orientation.z = 0
        pose_cam_frame.pose.orientation.w = 1
        pose_map_frame = self.tf_listener.transformPose(self.map_frame, pose_cam_frame)
        print("Quaternion: x=%.2f y=%.2f z=%.2f w=%.2f" % (pose_map_frame.pose.orientation.x, pose_map_frame.pose.orientation.y, 
                                                           pose_map_frame.pose.orientation.z, pose_map_frame.pose.orientation.w))
        self.parent.publish_goal_quat(pose_map_frame.pose.position.x, pose_map_frame.pose.position.y, pose_map_frame.pose.position.z,
                                      pose_map_frame.pose.orientation.x, pose_map_frame.pose.orientation.y, pose_map_frame.pose.orientation.z, 
                                      pose_map_frame.pose.orientation.w)

    def commit_gate(self, transform1, transform2):
        pass
        #TODO: complete this function

    def transform_to_map_frame(self, transform, header):
        """! Transforms a fiducial transform into the map frame.
        @param transform  Fiducial transform.
        @param header  Header from the FiducialTransformArray message
        @return  The pose in the map frame or None if the transformed failed.
        """
        pose_cam_frame = self.transform_to_pose_stamped(transform, header)
        try:
            pose_map_frame = self.listener.transformPose(self.map_frame, pose_cam_frame)
            return pose_map_frame
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None


def calc_dist(position1, position2):
    return sqrt((position2.x - position1.x)**2 + 
                (position2.y - position1.y)**2 +
                (position2.z - position1.z)**2)


def transform_to_pose_stamped(transform, header):
        """! Converts a fiducial transform to a geometry_msgs/PoseStamped message.
        @param transform  Fiducial transform.
        @param header  Header from the FiducialTransformArray message
        @return  PoseStamped message
        """
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = transform.translation.x
        pose.pose.position.y = transform.translation.y
        pose.pose.position.z = transform.translation.z
        pose.pose.orientation.x = transform.rotation.x
        pose.pose.orientation.y = transform.rotation.y
        pose.pose.orientation.z = transform.rotation.z
        pose.pose.orientation.w = transform.rotation.w
        return pose
