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
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Int8
from utils import *

#TODO: "Operators may at any point send a signal to the rover to abort the current attempt and
#       autonomously return to the previous post/gate or GNSS coordinate and stop within 10 m of it."

class MultipleGpsGoals:

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
        self.i = 1  # Index of the active goal. Initialized to 1 since 0 is the start position.
        self.goal_reached = False
        self.aruco_handler = ArucoHandler(self)
        self.continue_to_next = False
        self.return_to_previous = False

        # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
        self.origin_lat, self.origin_long = get_origin_lat_long()

        # Services
        self.continue_service = rospy.Service('goal_manager/continue', Empty, self.handle_continue_service)
        self.return_to_previous_service = rospy.Service('goal_manager/return_to_previous', Empty, self.handle_return_to_previous)

        self.state_pub = rospy.Publisher("/goal_manager/state", Int8, queue_size=1)  # 0 = autonomous navigation in progress, 1 = goal reached
        rospy.Subscriber('gps_goals_list', GpsGoalArray, self.sendMultipleGPSGoals)
        rospy.Subscriber('gps_goal_pose', PoseStamped, self.sendPoseGoal)
        rospy.Subscriber('fix', NavSatFix, self.fixCB)

    def run(self):
        """! Loop handling the activation of the aruco tag detection based on distance from the goal.
        """
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.curr_fix != None and self.curr_goal != None:
                dist = calc_distance(self.curr_fix.latitude, self.curr_fix.longitude,
                                        self.curr_goal.latitude, self.curr_goal.longitude)
                rospy.loginfo("Distance to goal: %.2f" % dist)
                if 0 < self.aruco_handler.mode < 2:
                    if not self.aruco_handler.enabled and dist < self.dist_threshold:
                        self.aruco_handler.enable(True)    
            r.sleep()

    def fixCB(self, data):
        """! Callback function for the GPS fix of the robot.
        @param data  sensor_msgs/NavSatFix message.
        """
        self.curr_fix = data

    def go_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
        # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
        x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
        # Create move_base goal
        self.publish_goal_euler(x, y, z, yaw, roll, pitch)

    def publish_goal_quat(self, x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        """! Publishes the goal to move_base as a 3D position and a quaternion for orientation.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = rospy.get_param('~frame_id','map')
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z =  z
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        # Send goal
        self.move_base.send_goal(goal, done_cb=self.goal_done_cb)
        status = self.move_base.get_goal_status_text()
        if status:
          rospy.loginfo(status)
        
    def publish_goal_euler(self, x, y, z, yaw, roll, pitch):
        """! Published the goal to move_base as a 3D position and euler angles for orientation.
        """
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.publish_goal_quat(x, y, z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    def sendMultipleGPSGoals(self, data):
        """! Callback function the array of GPS goals to reach in succession.
        @param data  goal_manager/GpsGoalArray message.
        """
        # TODO: during a return to previous, the rover only needs to stop within 10m. Should tag detection be ignored in that case?
        rospy.loginfo("Received multiple gps goals")
        self.i = 1
        while self.i < len(data.gps_goals):
            gpsGoal = data.gps_goals[self.i]
            rospy.loginfo("Sending goal {}: lat={} long={} type={}".format(self.i, gpsGoal.goal.latitude, gpsGoal.goal.longitude, gpsGoal.type))
            self.curr_goal = gpsGoal.goal
            self.sendGPSGoal(gpsGoal)
            success = self.wait_for_goal_reached()
            if success:
                rospy.loginfo("Goal %d reached" % self.i)
            if success:
                self.wait_for_continue()
                self.i += 1
        self.curr_goal = None
        rospy.loginfo("Multiple waypoint trajectory completed.")

    def sendGPSGoal(self, data):
        """! Sends a single GPS goal as the active goal.
        @param data  goal_manager/GpsGoal message.
        """
        try: 
            self.aruco_handler.set_mode(data.type)
            self.goal_reached = False
            self.go_gps_goal(data.goal.latitude, data.goal.longitude)
        except:
            rospy.logwarn('*Error sending command : \n'+traceback.format_exc())

    def sendPoseGoal(self, data):
        """! Callback function to send a single PoseStameped message as a goal.
        @param data  geometry_msgs/PoseStamped message.
        """
        #TODO: is this still needed? If not, remove it.
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
        rospy.loginfo('move_base status: %s' % GoalStatus.to_string(state))
        if not self.aruco_handler.enabled:
            self.goal_reached = True
        elif self.aruco_handler.goal_in_progess:
            self.goal_reached = True
            self.aruco_handler.enable(False)
        self.curr_goal = None

    def wait_for_goal_reached(self):
        """! Function that loops until the goal is reached. Returns if a "return to previous goal" action is sent.
        @return Boolean indicating if the goal was reached or if it was preempted by the "return to previous goal" action.
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.goal_reached:
            if self.return_to_previous:
                self.return_to_previous = False
                self.i -= 1
                rospy.loginfo("Returning to goal %d" % self.i)
                return False
            self.state_pub.publish(Int8(0))
            r.sleep()
        return True

    def wait_for_continue(self):
        """! Function that loops until the user sends a "continue to next goal" once the goal is reached.
        """
        rospy.loginfo("Waiting for user input before continuing to next goal.")
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.continue_to_next:
            if self.return_to_previous:
                self.return_to_previous = False
                self.i -= 2
                rospy.loginfo("Returning to goal %d" % (self.i+1))
                return
            self.state_pub.publish(Int8(1))
            r.sleep()
        self.continue_to_next = False

    def handle_continue_service(self, req):
        """! Callback that handles the requests for the "continue" service
        @param req  Service request (not used).
        @return Empty service response
        """
        if self.goal_reached:
            self.continue_to_next = True
        return EmptyResponse()

    def handle_return_to_previous(self, req):
        """! Callback that handles the requests for the "continue" service
        @param req  Service request (not used).
        @return Empty service response
        """
        if self.i - 1 >= 0:
            self.return_to_previous = True
        return EmptyResponse()


if __name__ == '__main__':
    try:
        multiple_gps_goals = MultipleGpsGoals()
        multiple_gps_goals.run()
    except rospy.ROSInterruptException:
        pass
