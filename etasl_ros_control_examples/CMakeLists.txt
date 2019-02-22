cmake_minimum_required(VERSION 2.8.3)
project(etasl_ros_control_examples)

find_package(catkin)

catkin_package()

## Installation
install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/target_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
