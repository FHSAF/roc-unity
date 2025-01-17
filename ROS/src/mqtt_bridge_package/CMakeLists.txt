cmake_minimum_required(VERSION 3.0.2)
project(mqtt_bridge_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/mqtt_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
