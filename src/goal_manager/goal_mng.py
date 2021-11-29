#!/usr/bin/env python

import rospy
import actionlib
import tf
import traceback

from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from goal_manager.msg import list_gps_goals
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

from utils import *



class MultipleGpsGoals():

    def __init__(self):
        rospy.init_node('goal_manager', anonymous=True)

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        rospy.loginfo("Connected.")

        rospy.Subscriber('list_gps_goals', list_gps_goals, self.sendMultipleGPSGoals)
        rospy.Subscriber('gps_goal_pose', PoseStamped, self.sendPoseGoal)
        rospy.Subscriber('gps_goal_fix', NavSatFix, self.sendGPSGoal)

        # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
        self.origin_lat, self.origin_long = get_origin_lat_long()


    def go_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
        # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
        x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
        # Create move_base goal
        self.publish_goal(x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch)
        
    
    def publish_goal(self, x=0, y=0, z=0, yaw=0, roll=0, pitch=0):
        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = rospy.get_param('~frame_id','map')
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z =  z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        rospy.loginfo('Executing move_base goal to position (x,y) %s, %s, with %s degrees yaw.' %
                (x, y, yaw))
        rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        # Send goal
        self.move_base.send_goal(goal)
        rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
        status = self.move_base.get_goal_status_text()
        if status:
          rospy.loginfo(status)

        # Wait for goal result
        self.move_base.wait_for_result()
        rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
        status = self.move_base.get_goal_status_text()
        if status:
          rospy.loginfo(status)


    def sendMultipleGPSGoals(self, data):
        for gpsGoal in data.gpsgoals:
            self.sendGPSGoal(gpsGoal)

    def sendGPSGoal(self, data):
        try:
            self.go_gps_goal(data.latitude, data.longitude)
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




if __name__ == '__main__':
    #gps_goal.ros_main()

    MultipleGpsGoals()
    rospy.spin()


