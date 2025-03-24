cmake_minimum_required(VERSION 3.5)
project(planning)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-std=c++17)
add_compile_options(-std=c++17)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find the path to the bullet folder
if(DEFINED ENV{BULLET_INSTALL_PATH})
  set(BULLET_ROOT $ENV{BULLET_INSTALL_PATH})
  set(BULLET_INCLUDE_DIRS $ENV{BULLET_INSTALL_PATH}/include/bullet)
  message(STATUS "Bullet path found" $ENV{BULLET_INSTALL_PATH})
endif()

# Find all the required packages starting with roscpp
find_package(catkin REQUIRED COMPONENTS
  roscpp roslib
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(console_bridge REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(kinosplines REQUIRED)
find_package(learning_sensitivity REQUIRED)
find_package(ompl REQUIRED)
find_package(robots REQUIRED)
find_package(services_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(utils REQUIRED)

set(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_ROOT_DIR}/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# Find bullet packages
list(APPEND BULLET_LIB_NAMES BulletRoboticsGUI
  BulletExampleBrowserLib
  BulletRobotics
  BulletFileLoader
  BulletWorldImporter
  BulletSoftBody
  BulletDynamics
  BulletCollision
  BulletInverseDynamicsUtils
  BulletInverseDynamics
  LinearMath
  OpenGLWindow
  gwen
  BussIK
  Bullet3Common)

foreach(lib_name ${BULLET_LIB_NAMES})
  find_library(LIB_PATH ${lib_name} ${BULLET_ROOT}/lib NO_DEFAULT_PATH)
  list(APPEND BULLET_LIBS ${LIB_PATH})
  unset(LIB_PATH CACHE)
  message(STATUS "Bullet lib compiled: " ${lib_name})
endforeach()

# Create a package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${kinosplines_INCLUDE_DIRS}
  ${learning_sensitivity_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
  ${robots_INCLUDE_DIRS}
  ${services_msgs_INCLUDE_DIRS}
  ${utils_INCLUDE_DIRS}
)

# Declare bullet library
add_library(Bullet3 INTERFACE)
target_include_directories(Bullet3 INTERFACE ${BULLET_INCLUDE_DIRS})
target_link_libraries(Bullet3 INTERFACE
                      ${BULLET_LIBS}
                      ${CMAKE_DL_LIBS}
                      ${catkin_LIBRARIES}
                      Threads::Threads)

add_library(${PROJECT_NAME}_bullet_lib STATIC
  src/bullet/PhysicsServers.cpp
  src/bullet/BulletClient.cpp
  src/bullet/RosFiles.cpp
)
target_link_libraries(${PROJECT_NAME}_bullet_lib PUBLIC 
                      Bullet3)
set_target_properties(${PROJECT_NAME}_bullet_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Declare state spaces library
add_library(${PROJECT_NAME}_state_spaces_lib
  src/state_spaces/dubins/DubinsUnicycleSpace.cpp
  src/state_spaces/kinospline/KinosplineStateSpace.cpp
  src/state_spaces/kinospline/KinoQuadrotorSpace.cpp
  src/state_spaces/kinospline/KinoUnicycleSpace.cpp
)
target_link_libraries(${PROJECT_NAME}_state_spaces_lib
  ${kinosplines_LIBRARIES}
  ${robots_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
  Eigen3::Eigen
  ompl::ompl  
)

# Declare samplers library
add_library(${PROJECT_NAME}_samplers_lib
  src/samplers/DubinsStateSampler.cpp
  src/samplers/KinosplineStateSampler.cpp
)
target_link_libraries(${PROJECT_NAME}_samplers_lib
  ${catkin_LIBRARIES}
  ${robots_LIBRARIES}
  ${PROJECT_NAME}_state_spaces_lib
  ompl::ompl  
)

# Declare objectives library
add_library(${PROJECT_NAME}_objectives_lib
  src/opt_objectives/KS_time_obj.cpp
  src/opt_objectives/Sensi_obj.cpp
)
target_link_libraries(${PROJECT_NAME}_objectives_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_state_spaces_lib
  ompl::ompl
)

# Declare state checker library
add_library(${PROJECT_NAME}_state_checker_lib
  src/state_checker/RobustStateValidityChecker.cpp
)
target_link_libraries(${PROJECT_NAME}_state_checker_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_bullet_lib
  ${PROJECT_NAME}_state_spaces_lib
  ${utils_LIBRARIES}
  ompl::ompl
)

# Declare visual library
add_library(${PROJECT_NAME}_visual_lib
  src/visual_tools/BulletVisualTools.cpp
)
target_link_libraries(${PROJECT_NAME}_visual_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_bullet_lib
  ${PROJECT_NAME}_state_spaces_lib
  ${utils_LIBRARIES}
  ompl::ompl
)

# Declare motion validators library
add_library(${PROJECT_NAME}_motion_validator_lib
  src/motion_validator/DiscreteRobustMotionValidator.cpp
)
target_link_libraries(${PROJECT_NAME}_motion_validator_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_state_checker_lib
  ${PROJECT_NAME}_state_spaces_lib
  ${robots_LIBRARIES}
  Eigen3::Eigen
  ompl::ompl
)

# Declare planners library
add_library(${PROJECT_NAME}_planners_lib
  src/planners/FMT.cpp
  src/planners/Generator.cpp
  src/planners/RandUPRRT.cpp
  src/planners/RRTstar.cpp
  src/planners/RRT.cpp
  src/planners/R_SAFMT_NN.cpp
  src/planners/R_SARRT.cpp
  src/planners/R_SARRT_NN.cpp
  src/planners/R_SARRTstar.cpp
  src/planners/R_SARRTstar_NN.cpp
  src/planners/SARRT.cpp
  src/planners/SARRTstar.cpp
  src/planners/SST.cpp
  src/planners/SSTstar.cpp
  src/planners/R_SASST.cpp
  src/planners/R_SASSTstar.cpp
  src/planners/TRRT.cpp
)
target_link_libraries(${PROJECT_NAME}_planners_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_state_spaces_lib
  ${PROJECT_NAME}_visual_lib
  Eigen3::Eigen
  ompl::ompl
)

# Declare post processing library
add_library(${PROJECT_NAME}_post_process_lib
  src/post_processing/Shortcut.cpp
  src/post_processing/Smooth.cpp
  src/post_processing/ExtendedShortcut.cpp
  src/post_processing/NloptLocalopt.cpp
  src/post_processing/STOMP.cpp
  src/post_processing/stomp/stomp.cpp
  src/post_processing/stomp/utils.cpp
)
target_link_libraries(${PROJECT_NAME}_post_process_lib
  ${catkin_LIBRARIES}
  ${robots_LIBRARIES}
  ${PROJECT_NAME}_state_spaces_lib
  Eigen3::Eigen
  ompl::ompl
)

# Declare context library
add_library(${PROJECT_NAME}_context_lib
  src/context/PlanningContext.cpp
)
target_link_libraries(${PROJECT_NAME}_context_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objectives_lib
  ${PROJECT_NAME}_motion_validator_lib
  ${PROJECT_NAME}_planners_lib
  ${PROJECT_NAME}_post_process_lib
  ${PROJECT_NAME}_samplers_lib
  ${PROJECT_NAME}_state_checker_lib
  ${PROJECT_NAME}_state_spaces_lib
  ${PROJECT_NAME}_visual_lib
  ${utils_LIBRARIES}
  Eigen3::Eigen
  ompl::ompl
)

# Declare context library
add_library(${PROJECT_NAME}
  src/Plan.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${PROJECT_NAME}_context_lib
  ompl::ompl
)

# Declare the executable and its dependancies
add_executable(plan src/Plan.cpp)
target_link_libraries(plan ${catkin_LIBRARIES} ${Boost_LIBRARIES} ompl::ompl ${PROJECT_NAME}_context_lib)
install(TARGETS plan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})