cmake_minimum_required(VERSION 2.8.3)
project(etasl_ros_control_cmd)

find_package(catkin)

catkin_package()

## Installation
install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
  scripts/etasl_pose_publisher.py
  scripts/jointstate_grapher.py
  scripts/smach_task.py
  scripts/speed_pub.py
  scripts/zivid_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
