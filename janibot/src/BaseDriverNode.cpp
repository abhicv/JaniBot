#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#define WHEEL_VELOCITY 30.0

std_msgs::Float64 wheel_bl;
std_msgs::Float64 wheel_br;
std_msgs::Float64 wheel_fl;
std_msgs::Float64 wheel_fr;

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("cmd_vel: x: %f, y: %f, rotz: %f\n", msg->linear.x, msg->linear.y, msg->angular.z);
    
    if(msg->linear.x == 0.0 && msg->linear.y == 0.0) //stop
    {
        wheel_bl.data = 0.0;
        wheel_br.data = 0.0;
        wheel_fl.data = 0.0;
        wheel_fr.data = 0.0;
    }
    else if(msg->linear.x > 0.0 && msg->linear.y == 0.0) //forward
    {
        wheel_bl.data = -WHEEL_VELOCITY;
        wheel_br.data = WHEEL_VELOCITY;
        wheel_fl.data = -WHEEL_VELOCITY;
        wheel_fr.data = WHEEL_VELOCITY;
    }
    else if(msg->linear.x < 0.0 && msg->linear.y == 0.0) //backward
    {
        wheel_bl.data = WHEEL_VELOCITY;
        wheel_br.data = -WHEEL_VELOCITY;
        wheel_fl.data = WHEEL_VELOCITY;
        wheel_fr.data = -WHEEL_VELOCITY;
    }
    
    if(msg->angular.z > 0.0) //rotate right
    {
        wheel_bl.data = WHEEL_VELOCITY;
        wheel_br.data = WHEEL_VELOCITY;
        wheel_fl.data = WHEEL_VELOCITY;
        wheel_fr.data = WHEEL_VELOCITY;
    }
    else if(msg->angular.z < 0.0) //rotate left
    {
        wheel_bl.data = -WHEEL_VELOCITY;
        wheel_br.data = -WHEEL_VELOCITY;
        wheel_fl.data = -WHEEL_VELOCITY;
        wheel_fr.data = -WHEEL_VELOCITY;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BaseDriverNode");
    
    ros::NodeHandle nodeObj;
    
    ros::Publisher wheel_bl_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/wheel_joint_back_left_position_controller/command", 100);
    ros::Publisher wheel_br_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/wheel_joint_back_right_position_controller/command", 100);
    ros::Publisher wheel_fl_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/wheel_joint_front_left_position_controller/command", 100);
    ros::Publisher wheel_fr_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/wheel_joint_front_right_position_controller/command", 100);
    
    ros::Subscriber cmd_vel_subscriber = nodeObj.subscribe("/cmd_vel", 10, CmdVelCallback);
    
    ros::Rate loopRate(100);
    
    wheel_bl.data = 0.0;
    wheel_br.data = 0.0;
    wheel_fl.data = 0.0;
    wheel_fr.data = 0.0;
    
    while(ros::ok())
    {
        wheel_bl_cmd_publisher.publish(wheel_bl);
        wheel_br_cmd_publisher.publish(wheel_br);
        wheel_fl_cmd_publisher.publish(wheel_fl);
        wheel_fr_cmd_publisher.publish(wheel_fr);
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}
