cmake_minimum_required(VERSION 2.8.3)
project(way_points_manager)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS scripts/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)