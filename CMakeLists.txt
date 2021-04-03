cmake_minimum_required(VERSION 2.8.3)
project(nav_sim)

find_package(catkin
  REQUIRED COMPONENTS
    move_base
    rospy
    nav_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    nav_msgs
    geometry_msgs
)
