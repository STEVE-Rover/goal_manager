cmake_minimum_required(VERSION 2.8.3)

project(goal_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  actionlib
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
  message_generation
)

add_message_files(
  FILES
  GpsGoal.msg
  GpsGoalArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy tf actionlib actionlib_msgs sensor_msgs std_msgs geometry_msgs move_base_msgs message_runtime
)