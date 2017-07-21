cmake_minimum_required(VERSION 2.8.3)
project(amiro_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
)

add_message_files(
  FILES
  UInt16MultiArrayStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#   INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
#   LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    nav_msgs
  DEPENDS
    message_runtime
)
