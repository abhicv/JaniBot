cmake_minimum_required(VERSION 2.8.3)
project(jani_arm_control)

## Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.3 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_ros
  pcl_conversions
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_ros_planning_interface
    moveit_visual_tools
    pcl_ros
    pcl_conversions
    sensor_msgs
    
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
##add_library(${PROJECT_NAME}
 ## src/robotarm_moveit_interface.cpp
##)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(arm_control src/arm_control.cpp)
add_executable(arm_control2 src/arm_control2.cpp)
target_link_libraries(arm_control ${catkin_LIBRARIES})
target_link_libraries(arm_control2 ${catkin_LIBRARIES})
target_link_libraries(arm_control2 ${PCL_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
