cmake_minimum_required(VERSION 3.5)
project(learning_sensitivity)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-std=c++17)
add_compile_options(-std=c++17)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
#############################################
# Add libtorch #
#############################################
set(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../")
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_ROOT_DIR}/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include ${TORCH_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/SensitivityNN.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TORCH_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
 
install(
DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})