cmake_minimum_required(VERSION 2.8.3)
project(take_images)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/camera_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
