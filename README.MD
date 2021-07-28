Janitorial Robot development repository for ArtPark Robotics Challenge


#### Instructions to use the realsense plugin 
Clone this repository https://github.com/pal-robotics/realsense_gazebo_plugin into `{catkin workspace}/src`

run `source devel/setup.bash` 

run `catkin_make` , this sets up the `librealsense_gazebo_plugin.so` used in the controller

and as always , run `source devel/setup.bash` for everything to be visible 

after this , just run the thing , it'll work (Hopefully)! 


### Instructions to use octomap
You have to install [octomap_ros](http://wiki.ros.org/octomap_ros) , [octomap_msgs](http://wiki.ros.org/octomap_msgs) and [octomap_server](http://wiki.ros.org/octomap_msgs), to run octomap_temp

The octomap_temp contains the launch file to launch the octomap mapping. 
However , the node odom_transform_publisher must be run before running it and filtered cloud.odom_transform_publisher provides the tf transformation between the base_frame and odom frames, which octomap uses to build its map, and filtered cloud voxelgridfilters the cloud down to make it easier for octomap to process

`rosrun odom_transform_publisher odom_trasnform_publisher_node`
and 
`rosrun filtered_cloud filtered_cloud_node`

The output of filtered_cloud node is taken as input by the launch file in ocotmap_temp [Which you can modify if you want]


### Robot Joints Manager
This is , by no means a useful node , but it does have some temporary value it sets a predefined pose for the robot
to operate run the node 
and then run `rosservice call robot_joints_manager set_predefined_pos [value]` where value is either 1 or zero. The source code may be edited to bring in more changes.




