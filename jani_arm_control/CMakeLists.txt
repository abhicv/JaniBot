cmake_minimum_required(VERSION 2.8.3)
project(jani_arm_control)
set(YOUR_DIRECTORY "~/artpark/src/jani_arm_control/src")
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
  tf2
  tf2_ros
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_ros_planning_interface
    moveit_visual_tools
    pcl_ros
    pcl_conversions
    sensor_msgs
    tf2
    tf2_ros
    geometry_msgs
    
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${YOUR_DIRECTORY})


## Declare a C++ library
##add_library(${PROJECT_NAME}
 ## src/robotarm_moveit_interface.cpp
##
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(main src/main.cpp ${YOUR_DIRECTORY}/object_extraction.h ${YOUR_DIRECTORY}/dimension_extraction.h)
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(main ${PCL_LIBRARIES})

add_executable(arm_control src/arm_control.cpp)
target_link_libraries(arm_control ${catkin_LIBRARIES})






install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
