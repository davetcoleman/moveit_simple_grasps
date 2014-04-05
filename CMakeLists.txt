cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_grasps)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
)

add_action_files(DIRECTORY action FILES
  GenerateGrasps.action
)

generate_messages(DEPENDENCIES
  geometry_msgs actionlib_msgs std_msgs moveit_msgs)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_visual_tools
  INCLUDE_DIRS include
)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# Grasp Generator Library
add_library(${PROJECT_NAME}
  src/moveit_simple_grasps.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Grasp Filter Library
add_library(${PROJECT_NAME}_filter
  src/grasp_filter.cpp
)
target_link_libraries(${PROJECT_NAME}_filter 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test executable
add_executable(${PROJECT_NAME}_server src/moveit_simple_grasps_server.cpp)
target_link_libraries(${PROJECT_NAME}_server
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test executable
add_executable(${PROJECT_NAME}_test src/moveit_simple_grasps_test.cpp)
target_link_libraries(${PROJECT_NAME}_test 
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Install
install(TARGETS 
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_filter 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)




