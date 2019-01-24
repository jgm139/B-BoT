cmake_minimum_required(VERSION 2.8.3)
project(yolo_detection_obj)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  yolo_msg
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolo_detection_obj
#  CATKIN_DEPENDS rospy sensor_msgs yolo_msg
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/detection_objects.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
