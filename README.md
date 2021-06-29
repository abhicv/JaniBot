# JaniBot

#Jani_arm_control contains the moveit control of the arm.

#the code files are located in src folder of Jani_arm_control

#arm_control.cpp contains code for moving the arm to a particular position (if it is in its taks space) and some other stuff

#arm_control2.cpp contains code for visualizing the point cloud captured through realsense using pcl visualizer.Its not working yet.

#The launch folder of jani_arm_control contains many launch files but 3 are important."my_gazebo.launch" "my_move_group.launch" "my_rviz.launch".These files
should be launched in this order. If your laptop is strong enough then u can combine these 3 into one and launch it together But do combine it
in the order mentioned.

#If u just want to play with the arm in rviz then launch the "display.launch" in robot_description.

#These are the plans for near future:

-> To spawn the cylinder on the counter of config_2.world and segment it.

-> To plan the pose to grasp that cylinder

-> To spawn the cup like object on the counter of config_2.world and segment it.

-> To plan the pose to grasp that object

-> To make the arm distinguish between the cylinder and the cone (either using machine learning or some other technique) and then proceed
accordingly to pick it up

-> To spawn some objects in its way and plan the arm to avoid the object while picking up the trash

-> To determine the task space of the arm so that we can always position the base in such a way that the trash lies in its task space.

-> To tune the pid values of controller using the motor's parameters hence making it realistic (As of now I have included velocity limit but haven't consider acceleration)
