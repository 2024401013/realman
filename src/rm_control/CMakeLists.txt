cmake_minimum_required(VERSION 3.0.2)
project(rm_control)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  trajectory_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  rm_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    actionlib 
    actionlib_msgs 
    control_msgs 
    trajectory_msgs 
    sensor_msgs 
    roscpp 
    rospy 
    std_msgs 
    rm_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(rm_control src/rm_control.cpp)

## Add dependencies
add_dependencies(rm_control ${rm_msgs_EXPORTED_TARGETS})

## Link libraries
target_link_libraries(rm_control ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS rm_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
