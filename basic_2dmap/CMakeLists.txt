cmake_minimum_required(VERSION 3.0.2)
project(pcl_testing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	     pcl_ros
	     roscpp
	     rospy
	     sensor_msgs
	     std_msgs
	     nav_msgs
	     geometry_msgs
	     tf2
	     tf2_ros 
	     pcl_conversions
	     )

catkin_package()


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(pcl_testing_node src/mapping.cpp)


target_link_libraries( pcl_testing_node
   ${catkin_LIBRARIES}
 )


