cmake_minimum_required(VERSION 3.10.2)
project(sobits_common)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamixel_sdk
  controller_manager
  hardware_interface
  joint_limits_interface
  sobits_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rospy
  dynamixel_sdk
  controller_manager
  hardware_interface
  joint_limits_interface
  sobits_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_servo_control src/dynamixel/dynamixel_servo_control.cpp
                                       src/dynamixel/dynamixel_joint_control.cpp
                                       src/dynamixel/dynamixel_port_control.cpp
                                       src/dynamixel/dynamixel_setting.cpp)
add_dependencies(dynamixel_servo_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_servo_control ${catkin_LIBRARIES})
