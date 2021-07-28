#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "robot_joints_manager/SetPredefinedJoint.h"



/*
===================================================================================================
	This is a node which takes in predefined joint positions as a service, and sets 
	sets them properly 

	USAGE: Run the node 
	and run:
	 
	rosservice call /set_predefined_pose VALUE 
===================================================================================================
*/


class RobotArmControl{
	public:
	
		//Publisher for position  values 
		ros::Publisher link_one_cmd_publisher;
    	ros::Publisher link_two_cmd_publisher;
    	ros::Publisher link_three_cmd_publisher;
    	ros::Publisher link_end_cmd_publisher;
    	ros::Publisher cam_yaw_cmd_publisher;
    	ros::Publisher cam_pitch_cmd_publisher;
    	ros::Publisher arm_base_cmd_publisher;
		

		
		
		//Position values 
    	std_msgs::Float64 link_one_pos;
		std_msgs::Float64 link_two_pos;
		std_msgs::Float64 link_three_pos;
		std_msgs::Float64 link_end_pos;
		std_msgs::Float64 arm_base_pos;
		std_msgs::Float64 cam_yaw_pos;
		std_msgs::Float64 cam_pitch_pos;
	
	
/*
================================================================================================
	Under some other circumstances we might want to get rid of the /robot namespace 
	
	"I really want to make write better code , but I don't know how to, in cpp" -gandhi
================================================================================================
*/	
	
	//Constructor 
	RobotArmControl(){

		link_two_pos.data=0.0;
		link_three_pos.data=0.0;
		link_end_pos.data=0.0;
		arm_base_pos.data=0.0;
		cam_yaw_pos.data=0.0;
		cam_pitch_pos.data=0.0;
	
	}


	//Destructor
	~RobotArmControl()
	{
			
	}
	
	 
	//initialise publishers 
	void initPublishers(ros::NodeHandle& nodeObj)
	{
	
		link_one_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/link_1_joint_position_controller/command", 1);
		link_two_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/link_2_joint_position_controller/command", 1);
		link_three_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/link_3_joint_position_controller/command", 1);
		link_end_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/link_end_joint_position_controller/command", 1);
		arm_base_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/arm_base_joint_position_controller/command", 1);
		cam_yaw_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/camera_yaw_rotation_position_controller/command", 1);
		cam_pitch_cmd_publisher = nodeObj.advertise<std_msgs::Float64>("/robot/camera_pitch_rotation_position_controller/command", 1);
	
	}
	
	
	 
	void setLinkOnePos(float pos){
	     // These are just initialisation values ; arm will be moved by moveit
	     link_one_pos.data = pos;
	}


	void setLinkTwoPos(float pos){
		link_two_pos.data = pos;
	}


	void setLinkThreePos(float pos){
		link_three_pos.data=pos;
	}



	void setLinkEndPos(float pos){
		link_end_pos.data = pos;
	}



	void setCamYawJointPos(float pos){
		cam_yaw_pos.data = pos;
	}
	
	
	void setCamPitchPos(float pos){
		cam_pitch_pos.data = pos;
	}


	void setArmBaseJointPos(float pos){
		arm_base_pos.data = pos;
	}

	
	//Publishes everything 
	void publishAll(){
		arm_base_cmd_publisher.publish(arm_base_pos);
		link_one_cmd_publisher.publish(link_one_pos);
		link_two_cmd_publisher.publish(link_two_pos);
		link_three_cmd_publisher.publish(link_three_pos);
		link_end_cmd_publisher.publish(link_end_pos);
		cam_yaw_cmd_publisher.publish(cam_yaw_pos);
		cam_pitch_cmd_publisher.publish(cam_pitch_pos);
	}

	//Publishes all camera poses 
	void publishCamPoses(){
		cam_yaw_cmd_publisher.publish(cam_yaw_pos);
		cam_pitch_cmd_publisher.publish(cam_pitch_pos);
	}

	//Publishes all arm poses
	void publishArmPoses(){
		// I shall debug this later
		link_one_cmd_publisher.publish(link_one_pos);
		link_two_cmd_publisher.publish(link_two_pos);
		link_three_cmd_publisher.publish(link_three_pos);
		link_end_cmd_publisher.publish(link_end_pos);
	}

};

//global variable for arm control 
static RobotArmControl arm_control;

//array containing predefined default poses 
double default_poses[2][7];

//robot mode 
int mode=0;
int MAX_MODES=2;
std::string mode_names[2]={"default","scanning"};

//Set these values 
//Default poses 
//Initialise all the predefined poses
void initDefaultPoses()
{
	//default pose 1
	default_poses[0][0]=0.0;
	default_poses[0][1]=0.0;
	default_poses[0][2]=0.0;
	default_poses[0][3]=0.0;
	default_poses[0][4]=0.0;
	default_poses[0][5]=0.0;
	default_poses[0][6]=0.0;

	//scanning 
	default_poses[1][0]=5.5;
	default_poses[1][1]=1.6;
	default_poses[1][2]=0.0;
	default_poses[1][3]=0.0;
	default_poses[1][4]=1.0;
	default_poses[1][5]=-3.1415926535;
	default_poses[1][6]=3.1415926535;
}




 
// Sets on of the predefined poses
// Index is the index of the predefined pose   
void setPoses(int index)
{
	
	arm_control.setLinkOnePos(default_poses[index][0]);
	arm_control.setLinkTwoPos(default_poses[index][1]);
	arm_control.setLinkThreePos(default_poses[index][2]);
	arm_control.setLinkEndPos(default_poses[index][3]);
	arm_control.setCamPitchPos(default_poses[index][4]);
	arm_control.setCamYawJointPos(default_poses[index][5]);
	arm_control.setArmBaseJointPos(default_poses[index][6]);

}


//service that sets to a predefined robot pose 
bool changeRobotPose(robot_joints_manager::SetPredefinedJoint::Request &req ,robot_joints_manager::SetPredefinedJoint::Response &resp )
{
	mode=req.num%MAX_MODES;
	setPoses(mode);
	arm_control.publishAll();
	ROS_INFO("Configuration changed to : %s (%d)",mode_names[mode].c_str(),mode);
	return true;
}


//The server service here 
//This service sets a predefined robot pose 
bool changeRobotArmPose(std_msgs::Float64::ConstPtr& request,std_msgs::Float64::ConstPtr& response)
{
	ROS_INFO("Changing its pose");
	arm_control.publishCamPoses();
	return true;
}


//Change camera pose
bool changeRobotCameraPose(std_msgs::Float64::ConstPtr& request,std_msgs::Float64::ConstPtr& response)
{
	ROS_INFO("ChangeCameraPose");
	arm_control.publishCamPoses();
	return true;	
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_joint_manager_service");
    ros::NodeHandle nodeObj;
    arm_control.initPublishers(nodeObj);
    initDefaultPoses();
	ros::ServiceServer adjustarmjointpositions=nodeObj.advertiseService("set_predefined_pos",changeRobotPose);    
	ros::spin();
}


