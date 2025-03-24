cmake_minimum_required(VERSION 3.5)
project(robots)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-std=c++17)
add_compile_options(-std=c++17)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(learning_sensitivity REQUIRED)
find_package(services_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(utils REQUIRED)

set(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../")
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_ROOT_DIR}/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy services_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${learning_sensitivity_INCLUDE_DIRS}
  ${services_msgs_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
  ${utils_INCLUDE_DIRS}
)

# Install the sensitivity and dynamic servers
catkin_install_python(PROGRAMS scripts/servers/quadrotor_sensitivity_server.py scripts/servers/quadrotor_dynamic_server.py scripts/servers/quadrotor_localopt_server.py 
scripts/servers/unicycle_sensitivity_server.py scripts/servers/unicycle_dynamic_server.py scripts/servers/unicycle_localopt_server.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/Quadrotor.cpp
  src/Unicycle.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${learning_sensitivity_LIBRARIES} ${services_msgs_LIBRARIES} ${TORCH_LIBRARIES} ${utils_LIBRARIES} ompl::ompl)

install(
TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
 
install(
DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
        PATTERN ".svn" EXCLUDE)
