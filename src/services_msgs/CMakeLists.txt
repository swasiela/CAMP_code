cmake_minimum_required(VERSION 3.5)
project(services_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  DesiredState.msg
  GainsValues.msg
  InitRobotState.msg
)

add_service_files(
  FILES
  DynamicSrv.srv
  LocaloptSrv.srv
  SensitivitySrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

