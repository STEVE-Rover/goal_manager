#!/usr/bin/env python

import rospy
import tf
from math import sqrt, cos, pi
from copy import deepcopy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseStamped, Transform
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
        self.committed_post_pose = None
        self.committed_gate_pose = None
        self.duplicate_dist_thresh = rospy.get_param("~duplicate_dist_thresh", 0.25)
        self.enable_detections_service = rospy.ServiceProxy('/aruco_detect/enable_detections', SetBool)
        self.enable(False)
        rospy.loginfo("Starting Aruco Handler")
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducials_cb)

    def set_mode(self, mode):
        """! Sets the search mode according to the goal type.
        @param mode  Int representing the mode (nothing(0), post(1) or gate(2))
        """
        if 0 <= mode <= 2:
            self.mode = mode
        else:
            rospy.logerr("Invalid mode")

    def enable(self, state):
        """! Function that calls the aruco_ros service to enables/disables the tag detection.
        @param state  Bool representing on(True) or off(False)
        @return Boolean indicating if the service call was successful.
        """
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
        """! Callback function for the aruco tag detections.
        @param msg fiducial_msgs/FiducialTransformArray
        """
        if len(msg.transforms) == 0 or not self.is_fiducials_valid(msg):
            return
        msg.transforms = self.remove_duplicates(msg.transforms)
        if self.mode == 1:  # Post
            if len(msg.transforms) != 1:
                return
            elif not self.goal_in_progess:
                self.commit_post(msg.transforms[0].transform, msg.header)
            else:
                self.recommit_post(msg.transforms[0].transform, msg.header)

        elif self.mode == 2:  # Gate
            if len(msg.transform) != 2:
                return
            elif not self.goal_in_progess:
                self.commit_gate(msg.transform[0].transform, msg.transform[1].transform, msg.header)
            else:
                self.recommit_gate(msg.transform[0].transform, msg.transform[1].transform, msg.header)
        
    def is_fiducials_valid(self, fiducials):
        """! Checks the validity of detected fiducials based on the id and distance.
        @param fiducials fiducial_msgs/FiducialTransformArray
        @return Boolean indicating if the fiducials are valid or not.
        """
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
        """! Removes duplicates if multiple fiducials with the same id are found.
        @param msg fiducial_msgs/FiducialTransformArray
        @return The list of fiducials without duplicates.
        """
        ids = []
        filtered_fiducials = []
        for fiducial in fiducials:
            if fiducial.fiducial_id not in ids:
                filtered_fiducials.append(fiducial)
            ids.append(fiducial.fiducial_id)
        return filtered_fiducials


    def commit_post(self, transform, header):
        """! Saves a fiducial as a post goal to be sent to move_base.
        @param transform fiducial_msgs/FiducialTransform
        @param header std_msgs/Header of the transforms
        """
        rospy.loginfo("Post found! Creating goal.")
        pose_cam_frame = transform_to_pose_stamped(transform, header)
        # The transform is in the camera frame, with the Z axis pointing out of the lens
        pose_cam_frame.pose.position.z -= 1  # Offset the goal 1m in front of the post
        # Ignore the fiducial's orientation, we just want the rover to reach the position
        pose_cam_frame.pose.orientation.x = 0
        pose_cam_frame.pose.orientation.y = 0
        pose_cam_frame.pose.orientation.z = 0
        pose_cam_frame.pose.orientation.w = 1
        pose_map_frame = self.tf_listener.transformPose(self.map_frame, pose_cam_frame)
        self.committed_post_pose = pose_map_frame
        self.goal_in_progess = True
        self.parent.publish_goal_quat(pose_map_frame.pose.position.x, pose_map_frame.pose.position.y, pose_map_frame.pose.position.z,
                                      pose_map_frame.pose.orientation.x, pose_map_frame.pose.orientation.y, pose_map_frame.pose.orientation.z, 
                                      pose_map_frame.pose.orientation.w)

    def recommit_post(self, transform, header):
        """! Re-saves a fiducial as a post goal to be sent to move_base if the position is significantly different
        from where it was previously detected.
        @param transform fiducial_msgs/FiducialTransform
        @param header std_msgs/Header of the transforms
        """
        # TODO: test this function
        # TODO: Refactor to avoid duplicate code with commit_gate
        pose_cam_frame = transform_to_pose_stamped(transform, header)
        # The transform is in the camera frame, with the Z axis pointing out of the lens
        pose_cam_frame.pose.position.z -= 1  # Offset the goal 1m in front of the post
        # Ignore the fiducial's orientation, we just want the rover to reach the position
        pose_cam_frame.pose.orientation.x = 0
        pose_cam_frame.pose.orientation.y = 0
        pose_cam_frame.pose.orientation.z = 0
        pose_cam_frame.pose.orientation.w = 1
        pose_map_frame = self.tf_listener.transformPose(self.map_frame, pose_cam_frame)
        if calc_dist(pose_map_frame.pose.position, self.committed_post_pose.pose.position) > 0.5:
            rospy.loginfo("Recommitting post")
            self.committed_post_pose = pose_map_frame
            self.goal_in_progess = True
            self.parent.publish_goal_quat(pose_map_frame.pose.position.x, pose_map_frame.pose.position.y, pose_map_frame.pose.position.z,
                                        pose_map_frame.pose.orientation.x, pose_map_frame.pose.orientation.y, pose_map_frame.pose.orientation.z, 
                                        pose_map_frame.pose.orientation.w)

    def commit_gate(self, transform1, transform2):
        """! Saves a fiducial as a gate goal to be sent to move_base.
        @param transform1 fiducial_msgs/FiducialTransform of the first gate fiducial
        @param transform2 fiducial_msgs/FiducialTransform of the second gate fiducial
        @param header std_msgs/Header of the transforms
        """
        # TODO: test this function
        rospy.loginfo("Gate found! Creating goal.")
        midpoint = get_midpoint(transform1, transform2)
        pose_map_frame = self.tf_listener.transformPose(self.map_frame, midpoint)
        self.committed_gate_pose = pose_map_frame
        self.goal_in_progess = True
        self.parent.publish_goal_quat(pose_map_frame.pose.position.x, pose_map_frame.pose.position.y, pose_map_frame.pose.position.z,
                                      pose_map_frame.pose.orientation.x, pose_map_frame.pose.orientation.y, pose_map_frame.pose.orientation.z, 
                                      pose_map_frame.pose.orientation.w)

    def recommit_gate(self, transform1, transform2):
        """! Re-saves a fiducial as a gate goal to be sent to move_base if the position is significantly different
        from where it was previously detected.
        @param transform1 fiducial_msgs/FiducialTransform of the first gate fiducial
        @param transform2 fiducial_msgs/FiducialTransform of the second gate fiducial
        @param header std_msgs/Header of the transforms
        """
        # TODO: test this function
        # TODO: Refactor to avoid duplicate code with commit_gate
        midpoint = get_midpoint(transform1, transform2)
        pose_map_frame = self.tf_listener.transformPose(self.map_frame, midpoint)
        if calc_dist(pose_map_frame.pose.position, self.committed_gate_pose.pose.position) > 0.5:
            rospy.loginfo("Recommitting gate")
            self.committed_gate_pose = pose_map_frame
            self.goal_in_progess = True
            self.parent.publish_goal_quat(pose_map_frame.pose.position.x, pose_map_frame.pose.position.y, pose_map_frame.pose.position.z,
                                        pose_map_frame.pose.orientation.x, pose_map_frame.pose.orientation.y, pose_map_frame.pose.orientation.z, 
                                        pose_map_frame.pose.orientation.w)

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
    """! Calculates the euclidean distance between two 3D points
        @param position1  geometry_msgs/Point for the first point.
        @param position2  geometry_msgs/Point for the second point.
        @return  Euclidean distance.
    """
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

    
def get_midpoint(transform1, transform2):
    """! Calculates the midpoint of two transforms.
        @param transform1 fiducial_msgs/FiducialTransform of the first gate fiducial
        @param transform2 fiducial_msgs/FiducialTransform of the second gate fiducial
        @return  fiducial_msgs/FiducialTransform of the midpoint
        """
    middle = Transform()
    middle.translation.x = (transform1.translation.x + transform2.translation.x) / 2
    middle.translation.y = (transform1.translation.y + transform2.translation.y) / 2
    middle.translation.z = (transform1.translation.z + transform2.translation.z) / 2
    middle.pose.orientation.x = 0
    middle.pose.orientation.y = 0
    middle.pose.orientation.z = 0
    middle.pose.orientation.w = 1
    return middle

