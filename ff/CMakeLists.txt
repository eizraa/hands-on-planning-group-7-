cmake_minimum_required(VERSION 3.10)
project(turtlebot_simulation)

find_package(catkin REQUIRED
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy stonefish_ros
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
