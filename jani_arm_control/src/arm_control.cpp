#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cstdlib>
#include <ctime>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv)
{
// boiler_plate code
ros::init(argc,argv,"arm_control");
ros::NodeHandle n;
ros::AsyncSpinner spinner(1);
spinner.start();

// creating interface with move_group node and planning scene interface
moveit::planning_interface::MoveGroupInterface arm_interface("arm");
moveit::planning_interface::PlanningSceneInterface ps_interface;
arm_interface.setPoseReferenceFrame("base_link");
const moveit::core::JointModelGroup* joint_model_group =
      arm_interface.getCurrentState()->getJointModelGroup("arm");
      
// creating interface with rviz
namespace rvt=rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools mvt("world");
mvt.loadRemoteControl();
mvt.deleteAllMarkers();
mvt.trigger();
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to start");

// Planning a pose
bool result=false;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;


// Below block randomly samples a point and plans it. IF the plan is successful(i.e robot is able to reach to that point) then it exectes it. It keeps on searching for the point until it finds an executable one

while(result==false)
{
mvt.deleteAllMarkers();
mvt.trigger();
geometry_msgs::PoseStamped pose_random=arm_interface.getRandomPose();
mvt.publishAxis(pose_random.pose);
mvt.trigger();
ROS_INFO("Frame:%s",pose_random.header.frame_id.c_str());
arm_interface.setPoseTarget(pose_random);
result=arm_interface.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
ROS_INFO("The Plan %s",result ? "is successfull":"has failed");
}
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

// Visualising the pose in rviz
mvt.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
mvt.trigger();
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

//Execute the trajectory.You can see it execting in rviz
arm_interface.execute(my_plan);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");


//Below block attaches an object(cylinder) to the gripper in planning interface which then also displayed in rviz. This object will be attached throughout the planning.
mvt.deleteAllMarkers();
mvt.trigger();

moveit_msgs::CollisionObject cylinder;
cylinder.header.frame_id=arm_interface.getEndEffectorLink();
cylinder.id="cylinder_1";
shape_msgs::SolidPrimitive primitive;
primitive.type=primitive.CYLINDER;
primitive.dimensions.resize(2);
primitive.dimensions[primitive.CYLINDER_HEIGHT]=0.2;
primitive.dimensions[primitive.CYLINDER_RADIUS]=0.03;
cylinder.primitives.push_back(primitive);

tf2::Quaternion qt;
qt.setRPY(0,1.57,0);
qt=qt.normalize();
geometry_msgs::Pose cylinder_pose;
cylinder_pose.position.z=0.01;
cylinder_pose.position.y=0.15;
cylinder_pose.position.x=-0.045;
cylinder_pose.orientation.w=qt.getW();
cylinder_pose.orientation.x=qt.getX();
cylinder_pose.orientation.y=qt.getY();
cylinder_pose.orientation.z=qt.getZ();
cylinder.primitive_poses.push_back(cylinder_pose);
cylinder.operation=cylinder.ADD;

///std::vector<moveit_msgs::CollisionObject> collision_objects;
//collision_objects.push_back(cylinder);
std::vector<std::string> touch_links;
touch_links.push_back("gripper_finger_1_1");
touch_links.push_back("gripper_finger_2_1");
touch_links.push_back("gripper_finger_3_1");
touch_links.push_back("gripper_base_1");
ps_interface.applyCollisionObject(cylinder);
arm_interface.attachObject(cylinder.id,"gripper_base_1",touch_links);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

// Below block randomly samples a point and plans it with the object attached to the gripper.It keeps on searching for the point until it finds an executable one
result=false;
while(!result)
{
mvt.deleteAllMarkers();
mvt.trigger();
geometry_msgs::PoseStamped pose_random=arm_interface.getRandomPose();
mvt.publishAxis(pose_random.pose);
mvt.trigger();
ROS_INFO("Frame:%s",pose_random.header.frame_id.c_str());
arm_interface.setPoseTarget(pose_random);
result=arm_interface.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
ROS_INFO("The Plan %s",result ? "is successfull":"has failed");
}
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

// Visualising the pose in rviz
mvt.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
mvt.trigger();
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

//Go to a pose behind the object while avoiding.This doesn't work yet as we don't know the task space of robot yet so it is hard to get a point behing the object and make sure it is in task space or that the gripper is ble to rach that pose
/*
geometry_msgs::Pose target_pose;
target_pose.position.x=1.0;
target_pose.position.y=0.5;
target_pose.position.z=0.8;
target_pose.orientation.w=1;
mvt.publishAxis(target_pose);
mvt.trigger();
arm_interface.setPositionTarget(1.3,0.5,1);
bool bla=arm_interface.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
ROS_INFO("the plan %s",bla?"is successfull":"has failed");
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

if(bla)
{
mvt.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
mvt.trigger();
}

std::vector<std::string> object_ids;
object_ids.push_back(cylinder.id);
ps_interface.removeCollisionObjects(object_ids);
arm_interface.detachObject(cylinder.id);
mvt.deleteAllMarkers();
mvt.trigger();
tf2::Quaternion qt;
qt.setRPY(-1.57,0,0);
geometry_msgs::PoseStamped d_pose;
d_pose.header.frame_id=arm_interface.getPlanningFrame();
d_pose.pose.position.x=0.8738;
d_pose.pose.position.y=0.9042;
d_pose.pose.position.z=0.193;
d_pose.pose.orientation=tf2::toMsg(qt);
mvt.publishAxis(d_pose.pose);
mvt.trigger();
arm_interface.setPoseTarget(d_pose);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
result=arm_interface.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
ROS_INFO("The Plan %s",result ? "is successfull":"has failed");
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");


mvt.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
mvt.trigger();

mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
mvt.deleteAllMarkers(); 
mvt.trigger();

*/
ros::shutdown();
return 0;
}

