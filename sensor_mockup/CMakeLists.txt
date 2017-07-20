cmake_minimum_required(VERSION 2.8.3)
project(sensor_mockup)

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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(proximity_floor_mockup_node src/proximity_floor_mockup_node.cpp)

target_link_libraries(proximity_floor_mockup_node
  ${catkin_LIBRARIES}
)

add_executable(proximity_mockup_node src/proximity_mockup_node.cpp)

target_link_libraries(proximity_mockup_node
  ${catkin_LIBRARIES}
)

add_executable(rfid_mockup_node src/rfid_mockup_node.cpp)

target_link_libraries(rfid_mockup_node
  ${catkin_LIBRARIES}
)

add_dependencies(proximity_mockup_node sensor_mockup_gencpp sensor_mockup_gencpp)
add_dependencies(proximity_floor_mockup_node sensor_mockup_gencpp sensor_mockup_gencpp)