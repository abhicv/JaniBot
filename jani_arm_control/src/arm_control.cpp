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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <tf2_ros/transform_listener.h>
using namespace std;
//******The closest distance in -x direction from the gripper is 0.9m at which the arm will safely execute planning.


Eigen::Vector3f cross_product(Eigen::Vector3f v1,Eigen::Vector3f v2)
{
Eigen::Vector3f result;
result[0]=v1[1]*v2[2]-v1[2]*v2[1];
result[1]=v1[2]*v2[0]-v1[0]*v2[2];
result[2]=v1[0]*v2[1]-v1[1]*v2[0];
return result;
}


float dot_product(Eigen::Vector3f v1,Eigen::Vector3f v2)
{
float result=v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
return result;
}


Eigen::Vector3f normalize(Eigen::Vector3f v)
{
float factor=pow(v[0]*v[0]+v[1]*v[1]+v[2]*v[2],0.5);
v[0]=v[0]/factor;
v[1]=v[1]/factor;
v[2]=v[2]/factor;
return v;
}


void grasp_point(float axis_dir[], float centre_pt[],float dist,geometry_msgs::Pose& pose,float grasp_dir[])
{
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
geometry_msgs::TransformStamped transform_value;
ros::Duration(1.0).sleep();
transform_value=tfBuffer.lookupTransform("base_frame","modified_gripper_v3_1",ros::Time(0));
float x= -centre_pt[0]+transform_value.transform.translation.x;
float y= -centre_pt[1]+transform_value.transform.translation.y;
float z= -centre_pt[2]+transform_value.transform.translation.z;
cout<<transform_value.transform.translation.x<<endl<<transform_value.transform.translation.y<<endl<<transform_value.transform.translation.z;
cout<<endl<<x<<endl<<y<<endl<<z<<endl;
Eigen::Vector3f axis(axis_dir[0],axis_dir[1],axis_dir[2]);
Eigen::Vector3f centre_ptdir(x,y,z);
axis=normalize(axis);
centre_ptdir=normalize(centre_ptdir);
double angle=acos(dot_product(centre_ptdir,axis));
cout<<angle<<endl;
angle=(M_PI/2.0)-angle;
Eigen::Vector3f perpend;
perpend=cross_product(centre_ptdir,axis);
Eigen::Vector3f grasp_axis;
Eigen::Matrix3f A;
Eigen::Vector3f B;
A<<axis(0),axis(1),axis(2),centre_ptdir(0),centre_ptdir(1),centre_ptdir(2),perpend(0),perpend(1),perpend(2);
B<<0,cos(angle),0;
grasp_axis=A.colPivHouseholderQr().solve(B);
cout<<grasp_axis[0]<<endl<<grasp_axis[1]<<endl<<grasp_axis[2];
float lembda=dist/pow(grasp_axis[0]*grasp_axis[0]+grasp_axis[1]*grasp_axis[1]+grasp_axis[2]*grasp_axis[2],0.5);
pose.position.x=centre_pt[0]+lembda*grasp_axis[0];
pose.position.y=centre_pt[1]+lembda*grasp_axis[1];
pose.position.z=centre_pt[2]+lembda*grasp_axis[2];

Eigen::Vector3f el_init(1, 0, 0);
Eigen::Vector3f el_fin(-grasp_axis[0],-grasp_axis[1],-grasp_axis[2]);
el_init=normalize(el_init);
el_fin=normalize(el_fin);
if ((-0.01 < el_fin[0]+el_init[0] && el_fin[0]+el_init[0] <0.01) && (-0.01 <el_fin[1]+el_init[1] && el_fin[1]+el_init[1]<0.01) && (-0.01 < el_fin[2]+el_init[2] && el_fin[2]+el_init[2]<0.01))
{
perpend[0]=0;
perpend[1]=-1;
perpend[2]=0;
}
else
{
perpend=cross_product(el_init,el_fin);
perpend=normalize(perpend);
cout<<perpend[0]<<endl<<perpend[1]<<endl<<perpend[2];
cout<<endl<<"bla";
}
angle=acos(dot_product(el_init,el_fin));
cout<<endl<<angle;
pose.orientation.w=cos(angle/2);
pose.orientation.x=perpend[0]*sin(angle/2);
pose.orientation.y=perpend[1]*sin(angle/2);
pose.orientation.z=perpend[2]*sin(angle/2);
grasp_dir[0]=-grasp_axis[0];
grasp_dir[1]=-grasp_axis[1];
grasp_dir[2]=-grasp_axis[2];
}


void move_dist(geometry_msgs::Pose& curr_pose,float axis[], float& dist, int& np, float& interpolate_res,float& jump_thresh, moveit::planning_interface::MoveGroupInterface& arm_interface)
{
arm_interface.setPoseReferenceFrame("base_frame");
float dist_btw_points=dist/np;
vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(curr_pose);
geometry_msgs::Pose temp;
temp=curr_pose;
float lembda=dist_btw_points/pow(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2],0.5);
for (int i=0; i<np; i++)
{
temp.position.x=temp.position.x+ lembda*axis[0];
temp.position.y=temp.position.y+ lembda*axis[1];
temp.position.z=temp.position.z+ lembda*axis[2];
waypoints.push_back(temp);
}
moveit_msgs::RobotTrajectory traj;
arm_interface.computeCartesianPath(waypoints,interpolate_res,jump_thresh,traj);
arm_interface.execute(traj);
}


void move_gripper(moveit::planning_interface::MoveGroupInterface& gripper_interface,float& finger_angle)
{

//vector<double> curr_angle=gripper_interface.getCurrentJointValues();
vector<double> joint_values{finger_angle,finger_angle};
gripper_interface.setJointValueTarget(joint_values);
gripper_interface.move();

}

void pick(float axis_dir[],float centre_pt[],float& radius,float& pre_grasp_approach,float& pre_grasp_dist,float& post_grasp_retreat,int& num_of_waypoints,float& interpolate_res,float& jump_thresh,float& finger_openangle,float& finger_closeangle,moveit::planning_interface::MoveGroupInterface& arm_interface,moveit::planning_interface::MoveGroupInterface &gripper_interface,moveit::planning_interface::PlanningSceneInterface &ps_interface    
,moveit_visual_tools::MoveItVisualTools& mvt,moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
move_gripper(gripper_interface,finger_openangle);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

geometry_msgs::PoseStamped ps;
float grasp_dir[3]={0,0,0};
ps.header.frame_id="base_frame";
grasp_point(axis_dir,centre_pt,pre_grasp_approach,ps.pose,grasp_dir);
mvt.publishAxis(ps.pose);
mvt.trigger();
arm_interface.setPoseTarget(ps);
arm_interface.plan(plan);
arm_interface.execute(plan);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

move_dist(ps.pose, grasp_dir, pre_grasp_dist, num_of_waypoints, interpolate_res, jump_thresh, arm_interface);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

vector<string> obj;
obj.push_back("coke");
ps_interface.removeCollisionObjects(obj);

move_gripper(gripper_interface,finger_closeangle);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

float z_axis[3]={0,0,1};
move_dist( ps.pose, z_axis, post_grasp_retreat, num_of_waypoints, interpolate_res, jump_thresh, arm_interface);
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
}


void add_collisionobj(float axis_dir[],float centre_pt[],float& radius,float& height,moveit::planning_interface::PlanningSceneInterface& ps_interface)
{

moveit_msgs::CollisionObject cylinder;
cylinder.header.frame_id="base_frame";
cylinder.id="coke";
shape_msgs::SolidPrimitive primitive;
primitive.type=primitive.CYLINDER;
primitive.dimensions.resize(2);
primitive.dimensions[primitive.CYLINDER_HEIGHT]=height;
primitive.dimensions[primitive.CYLINDER_RADIUS]=radius;
cylinder.primitives.push_back(primitive);
geometry_msgs::Pose cylinder_pose;
cylinder_pose.position.x=centre_pt[0];
cylinder_pose.position.y=centre_pt[1];
cylinder_pose.position.z=centre_pt[2];
Eigen::Vector3f cylinder_dir(axis_dir[0],axis_dir[1],axis_dir[2]);
Eigen::Vector3f original_dir(0.0,0.0,1.0);
cylinder_dir.normalize();
original_dir.normalize();
Eigen::Vector3f axis;
axis=original_dir.cross(cylinder_dir);
axis.normalize();
double angle=acos(cylinder_dir.dot(original_dir));
cylinder_pose.orientation.w=cos(angle/2);
cylinder_pose.orientation.x=axis[0]*sin(angle/2);
cylinder_pose.orientation.y=axis[1]*sin(angle/2);
cylinder_pose.orientation.z=axis[2]*sin(angle/2);
cylinder.primitive_poses.push_back(cylinder_pose);
cylinder.operation=cylinder.ADD;
ps_interface.applyCollisionObject(cylinder);
}


int main(int argc, char** argv)
{
//initializing parameters
float height=0.107;
float radius=0.033;
float gripper_offset=0.13;
float pre_grasp_approach=0.20;  
float post_grasp_retreat=0.10;
int number_of_waypoints=4;
float interpolate_res=0.01;
float jump_thresh=0;
float pre_grasp_dist=pre_grasp_approach-gripper_offset;
float finger_openangle=2.66;
float finger_closeangle=-0.52;
//float finger_joint_tolerance=1.92;


// boiler_plate code for ros 
ros::init(argc,argv,"arm_control");
ros::NodeHandle n;
ros::AsyncSpinner spinner(1);
spinner.start();


//Setup moveit 
moveit::planning_interface::MoveGroupInterface arm_interface("arm");
const moveit::core::JointModelGroup* joint_model_group_arm =
      arm_interface.getCurrentState()->getJointModelGroup("arm");
moveit::planning_interface::PlanningSceneInterface ps_interface;    
moveit::planning_interface::MoveGroupInterface::Plan plan;
arm_interface.setPlannerId("RRT");
arm_interface.setPlanningTime(10.0);

moveit::planning_interface::MoveGroupInterface gripper_interface("gripper");
const moveit::core::JointModelGroup* joint_model_group_gripper =
      gripper_interface.getCurrentState()->getJointModelGroup("gripper");
//gripper_interface.setGoalJointTolerance(finger_joint_tolerance);


// creating interface with rviz
namespace rvt=rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools mvt("base_frame");
mvt.loadRemoteControl();
mvt.deleteAllMarkers();
mvt.trigger();
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to start");


//Initialize arm to home postition
arm_interface.setNamedTarget("home");
arm_interface.move();
mvt.prompt("Press 'next' in the RvizVisualToolsGui window to start");


//cout<<"THE PLANNING FRAME IS : "<<arm_interface.getPlanningFrame();
//The plannnig frame(i.e the frame in which arm plans the motion) is "base_frame"


/*float axis_dir[3];
axis_dir[0]=-0.002;
axis_dir[1]=0.022;
axis_dir[2]=-0.999;
float centre_pt[3];
centre_pt[0]=-1.038;
centre_pt[1]=-0.869;
centre_pt[2]=0.196; 
float dist=0.15;
*/

float centre_pt[3]={0.967,-0.209,0.055};
float axis_dir[3]={0,0,1};


add_collisionobj(axis_dir,centre_pt,radius,height,ps_interface);
pick(axis_dir,centre_pt,radius,pre_grasp_approach,pre_grasp_dist,post_grasp_retreat,number_of_waypoints,interpolate_res,jump_thresh,finger_openangle,finger_closeangle,arm_interface,gripper_interface,ps_interface,mvt,plan);

mvt.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

ros::Duration(1.0).sleep();


//arm_interface.setPoseTarget(curr_pose); //When we call setPoseTarget with a pose which is stamped than it takes that header frame.But if the pose is without reference frame then setPoseReferenceFrame() can be used to specify the reference frame.If we don't use setPoseReferenceFrame() with poses without reference frame it uses planning frame by default.
//**VERY IMPORTANT:- NEVER PASS A POSE WHICH HAS BEEN SET WITH RESPECT TO A MOVING FRAME BECAUSE PLANNING WILL FAIL.


mvt.deleteAllMarkers();
mvt.trigger();
ros::shutdown();
return 0;
}

