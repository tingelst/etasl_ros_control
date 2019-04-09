cmake_minimum_required(VERSION 2.8.3)
project(etasl_ros_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  etasl_ros_control_msgs
  expressiongraph_context
  expressiongraph_context_lua
  expressiongraph_solver_qpoases
  expressiongraph_collision
  controller_interface
  dynamic_reconfigure
  kdl_conversions
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package( Bullet REQUIRED)

# add_message_files(FILES
#   JointTorqueComparison.msg
# )

# generate_messages()

# generate_dynamic_reconfigure_options(
#   cfg/compliance_param.cfg
#   cfg/desired_mass_param.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES etasl_ros_controllers ${BULLET_LIBRARIES}
  CATKIN_DEPENDS
    etasl_ros_control_msgs
    expressiongraph_context
    expressiongraph_context_lua
    expressiongraph_solver_qpoases
    expressiongraph_collision
    controller_interface
    dynamic_reconfigure
    kdl_conversions
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
)

add_library(etasl_ros_controllers
  src/etasl_driver.cpp
  src/etasl_example_controller.cpp
)

# add_dependencies(franka_example_controllers
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${PROJECT_NAME}_gencpp
#   ${PROJECT_NAME}_gencfg
# )

target_link_libraries(etasl_ros_controllers PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(etasl_ros_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(etasl_ros_controllers PUBLIC
  include
)

## Installation
install(TARGETS etasl_ros_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES franka_example_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
# include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
#   RESULT_VARIABLE CLANG_TOOLS
# )
# if(CLANG_TOOLS)
#   file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
#   file(GLOB_RECURSE HEADERS
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
#   )
#   add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
#   add_tidy_target(franka_example_controllers
#     FILES ${SOURCES}
#     DEPENDS franka_example_controllers
#   )
# endif()