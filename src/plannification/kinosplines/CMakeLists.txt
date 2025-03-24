cmake_minimum_required(VERSION 3.5)
project(kinosplines)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-std=c++17)
add_compile_options(-std=c++17)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/kdtp_init.cpp
  src/kdtp_utils.cpp
  src/kdtp_dof.cpp
  src/kdtp_robot.cpp
  src/kdtp_state.cpp
  src/kdtp_spline.cpp
  src/kdtp_localpath.cpp
  src/KinoDynamicTime.cpp
)

### Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

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
