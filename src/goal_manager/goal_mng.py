#!/usr/bin/env python

import rospy
import actionlib
import tf
import traceback
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from goal_manager.msg import GpsGoal, GpsGoalArray
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from aruco_handler import ArucoHandler
from utils import *


class MultipleGpsGoals():

    def __init__(self):
        rospy.init_node('goal_manager', anonymous=True)

        rospy.loginfo("Waiting for move_base server")
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        self.rate = rospy.get_param("~rate", 2)  # Rate at which the distance to the goal is calculated
        self.dist_threshold = rospy.get_param("~dist_threshold", 5)  # Distance at which the aruco detection is enabled
        self.curr_fix = None
        self.curr_goal = None
        self.goal_reached = False
        self.aruco_handler = ArucoHandler(self)

        rospy.Subscriber('gps_goals_list', GpsGoalArray, self.sendMultipleGPSGoals)
        rospy.Subscriber('gps_goal_pose', PoseStamped, self.sendPoseGoal)
        rospy.Subscriber('fix', NavSatFix, self.fixCB)

        # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
        self.origin_lat, self.origin_long = get_origin_lat_long()

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.curr_fix != None and self.curr_goal != None and self.aruco_handler.get_mode > 0:
                dist = calc_distance(self.curr_fix.latitude, self.curr_fix.longitude,
                                     self.curr_goal.latitude, self.curr_goal.longitude)
                if dist < self.dist_threshold:
                    self.aruco_handler.enable(True)    
            r.sleep()

    def fixCB(self, data):
        self.curr_fix = data

    def go_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
        # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
        x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
        # Create move_base goal
        self.publish_goal_euler(x, y, z, yaw, roll, pitch)

    def publish_goal_quat(self, x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = rospy.get_param('~frame_id','map')
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z =  z
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        rospy.loginfo('Executing move_base goal to position (x,y) %s, %s.' %
                (x, y))
        rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        # Send goal
        self.move_base.send_goal(goal, done_cb=self.goal_done_cb)
        rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
        status = self.move_base.get_goal_status_text()
        if status:
          rospy.loginfo(status)

        # Wait for goal result
        
    def publish_goal_euler(self, x, y, z, yaw, roll, pitch):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.publish_goal_quat(x, y, z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    def sendMultipleGPSGoals(self, data):
        rospy.loginfo("Received multiple gps goals: {}".format(data.gps_goals))
        for i, gpsGoal in enumerate(data.gps_goals):
            rospy.loginfo("Sending goal {}: {} type:{}".format(i+1, gpsGoal.goal, gpsGoal.type))
            self.sendGPSGoal(gpsGoal)
            self.wait_for_goal_reached()
            #TODO: wait for a few seconds and change the LED color

    def sendGPSGoal(self, data):
        try: 
            self.aruco_handler.set_mode(data.type)
            self.goal_reached = False
            self.go_gps_goal(data.goal.latitude, data.goal.longitude)
        except:
            rospy.logwarn('*Error sending command : \n'+traceback.format_exc())

    def sendPoseGoal(self, data):
        lati = data.pose.position.y
        longi = data.pose.position.x
        z = data.pose.position.z
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.go_gps_goal(lati, longi, z=z, yaw=yaw, roll=roll, pitch=pitch)

    def goal_done_cb(self, state, result):
        """! Callback function for the move_base goal.
        @param state  Action state.
        @param result  Action result
        """
        rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(state))
        if not self.aruco_handler.override:
            self.goal_reached = True

        # if state == GoalStatus.SUCCEEDED:
        #     pass
        # elif state == GoalStatus.SUCCEEDED and self.retry == True:
        #     pass
        # elif state == GoalStatus.ABORTED:
        #     rospy.loginfo("Aborted goal")

    def wait_for_goal_reached(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.goal_reached:
            r.sleep()


if __name__ == '__main__':
    try:
        multiple_gps_goals = MultipleGpsGoals()
        multiple_gps_goals.run()
    except rospy.ROSInterruptException:
        pass
