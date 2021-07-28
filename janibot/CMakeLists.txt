cmake_minimum_required(VERSION 3.0.2)
project(janibot)

find_package(catkin REQUIRED
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  geometry_msgs
  pcl_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(BaseDriverNode src/BaseDriverNode.cpp)
target_link_libraries(BaseDriverNode ${catkin_LIBRARIES})

add_executable(DoorDetector src/DoorDetector.cpp)
target_link_libraries(DoorDetector ${catkin_LIBRARIES})