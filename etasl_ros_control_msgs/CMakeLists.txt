cmake_minimum_required(VERSION 2.8.3)
project(etasl_ros_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  Command.srv
  Picture.srv
  PegPos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs
)

#add_dependencies(activate_cmd_server
#  ${catkin_EXPORTED_TARGETS}
#)