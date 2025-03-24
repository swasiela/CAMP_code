cmake_minimum_required(VERSION 3.5)
project(utils)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-std=c++17)
add_compile_options(-std=c++17)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/project_ellipsoid.cpp
  src/transform.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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