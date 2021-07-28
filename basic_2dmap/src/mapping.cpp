#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <iostream>
#include <fstream>
#include <cmath>


#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
//#include <Eigen/Dense>
#include <pcl_ros/impl/transforms.hpp>





// DO NOT WRITE ANYTHING IN THE NEXT LINE  

ros::Publisher pub;
ros::Publisher gridpub;
geometry_msgs::Pose corrected_pose;
tf2_ros::Buffer tfbuffer;
geometry_msgs::Transform global_transform;



//occupancy_grid stuff 
nav_msgs::Odometry::Ptr odom_data(new nav_msgs::Odometry);
nav_msgs::OccupancyGrid::Ptr occupancy_grid(new nav_msgs::OccupancyGrid);
std::size_t width =  100;
std::size_t height = 100;
float resolution = 0.2f; 


//PointCloud stuff
pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud (new pcl::PointCloud<pcl::PointXYZ>);



// Mutex this 
// Position indication class 
class RobotPose2D{
	public:
		//position 
		double x;
		double y;
	
		//orientation 
		float theta;
		
		//didupdate 
		bool updated;
	
};

RobotPose2D robot_pose2d;





			// x in coord ; y in coord 
/*

		        | X
		        |
            	  Y_____|
		      ROBOT
		      ROBOT
		    EVILROBOT
		    
		    THETA HERE IS THE ANGLE WITH THE NORMAL 
*/

RobotPose2D relRotatePoints(double tf_x , double tf_y , double theta){
	RobotPose2D val;
	//val.x = y* cos(theta) -x*sin(theta);	//actual x in world 
	//val.y = y*sin(theta) + x* cos(theta);  //actual y in world
	
	//float corrected_theta = 1.57+theta;
	
	
	val.y=	tf_y*cos(theta) + tf_x*sin(theta);
	val.x = tf_x*cos(theta) - tf_y*sin(theta);
	
	return val;
}




void fillPosition(std::uint8_t value, std::size_t x, std::size_t y ){
	std::size_t pos =  width*y + x;
	if (pos>width*height-1)
	return;
	
	if (occupancy_grid->data[pos] == 0)
	{occupancy_grid->data[pos] = value;}
}








// Initialises occupancy grid with zero values 
void initOccupancyGrid(){
	//occupancy_grid (new nav_msgs::OccupancyGrid);	
	occupancy_grid->info.width=width;
	occupancy_grid->info.height=height;
	occupancy_grid->info.resolution= resolution;
	
	//geometry_msgs::Pose pose;
	
	occupancy_grid->info.origin.position.x = 0;
	occupancy_grid->info.origin.position.y = 0;
	occupancy_grid->info.origin.position.z = 0;
	
	occupancy_grid->info.origin.orientation.x = 0;
	occupancy_grid->info.origin.orientation.y = 0;
	occupancy_grid->info.origin.orientation.z = 0;
	occupancy_grid->info.origin.orientation.w = 0;
	
	occupancy_grid->info.map_load_time = ros::Time::now();
	
	for (std::size_t i=0;i<width*height;i++)
	occupancy_grid->data.push_back(0);
	
	
	//for (std::size_t i=0;i<10;i++)
	//occupancy_grid->data[i]=127;
	std::cout << "Initialised stuff";
	std::cout <<"occ grid size" <<occupancy_grid->data.size();
	std::cout <<"Fuck you \n";	
}





void clearOccupancyGrid(){
	for (std::size_t i=0;i<width*height;i++)
	occupancy_grid->data[i]=0;
}



// Odometry callback ; also does the conversion from quaternion to orientation 
void odometryCallBack(const nav_msgs::Odometry::ConstPtr& odommsg){
	//std::cout <<"Odometry Data" <<odommsg;
	
	std::ofstream myfile;
	//myfile.open("data.txt",std::ios::app);
	
	robot_pose2d.x = odommsg->pose.pose.position.x;
	robot_pose2d.y= odommsg->pose.pose.position.y;
	tf2::Quaternion quat;
	//tf2::fromMsg(odommsg->orientation , quat);
	double roll, pitch ,yaw; 
	double x,y;
	
	tf2::getEulerYPR(odommsg->pose.pose.orientation,yaw,pitch,roll);
	
	robot_pose2d.theta = yaw;
		
	robot_pose2d.updated=true; 
	
}










void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed (new pcl::PointCloud<pcl::PointXYZ>);
	
	
	
	
	pcl::fromROSMsg(*cloud_msg,*cloud);
	
	// Transform First 
	const std::string name = "base_link"; 
	//const pcl::PointCloud<pcl::PointXYZ>::Ptr test(cloud);
	
	pcl_ros::transformPointCloud(*cloud,*cloud_transformed,global_transform);
	
	
	
	// VoxelGrid Filter
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_transformed);
	sor.setLeafSize(0.1f,0.1f,0.1f);
	sor.filter(*cloud_filtered);

	
	
	// Gets a transformed cloud to base_link  
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.1,0.2);
	pass.filter(*processed_cloud);
	
	
	//fill occupancy grid
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*processed_cloud, output);
  	
  	
  	// Publish the data.
  	pub.publish (output);

}
 



void fillOccupancyGrid(){
	if (robot_pose2d.updated==false){
		return;
	}
	
	double x = robot_pose2d.x;
	double y = robot_pose2d.y;
	double theta = robot_pose2d.theta;
	
	double final_x = 0;
	double final_y = 0;
	
	double average_x = 0;
	double average_y = 0;
		
	for (std::size_t i=0;i<processed_cloud->size();i++){
		//do tranformation
		//Fill grid 
		// Find orientation and do rest of styff 
		RobotPose2D val = relRotatePoints(processed_cloud->points[i].x,processed_cloud->points[i].y,theta);
		
		final_x = val.x + x;
		final_y = val.y + y;
		
		
		//final_x = processed_cloud->points[i].x+x;
		//final_y = processed_cloud->points[i].y+y;
		
		int m = final_x/resolution;
		int n = final_y/resolution;
		
		average_x+=processed_cloud->points[i].y;
		average_y+=processed_cloud->points[i].x;
		
		
		if ((m>0&&m<100.0)&&(n>0&&n<100.0))
		{	fillPosition(50,(std::size_t)m,(std::size_t)n);}
			
	}
	
	float  p  = (x/resolution);
	float  q =  (y/resolution);
	
	
	average_x/=((float)processed_cloud->size());
	average_y/=((float)processed_cloud->size());
	
	//
	
	//std::cout<<"robot_global_x"<<x<<"robot_global_y"<<y;
	//std::cout<<"average_data_x"<<average_x<<"average_data_y"<<average_y;
	
	
	//ROS_INFO("tes");
	ROS_INFO(" robot_global_x : %f robot_global_y: %f",x,y);
	ROS_INFO("robot_global_theta: %f",theta);
	ROS_INFO(" average_data_x : %f average_data_y: %f",average_x,average_y);
	
	
	//ROS_INFO(" average_x  : %f average_y: %f",average_x,average_y);	
	if (p>0&&q>0)
	{fillPosition(240,(std::size_t)p,(std::size_t)q);}
	robot_pose2d.updated=false;
}











int main(int argc, char** argv){
	// Initialize ROS
  	ros::init (argc, argv, "my_pcl_tutorial");
  	ros::NodeHandle nh;
  	tf2_ros::TransformListener tfListener(tfbuffer);
  	geometry_msgs::TransformStamped transformStamped;
  		
  		
  		

	//*occupancy_grid = new nav_msgs::OccupancyGrid; 
	//odom_data (new nav_msgs::Odometry);
	initOccupancyGrid();	
		
	
	
  	// Create a ROS subscriber for the input point cloud
  	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, pointCloudCallback);
	ros::Subscriber odom_sub = nh.subscribe("/odom",1,odometryCallBack);
	
	
  	// Create a ROS publisher for the output point cloud
  	pub = nh.advertise<sensor_msgs::PointCloud2> ("processed_cloud", 1);
  	gridpub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid",1);


  	// Spin
  	
  	while (ros::ok())
  	{
  		ros::spinOnce();
  		
  		try{
  			transformStamped = tfbuffer.lookupTransform("base_link","camera_depth_optical_frame",ros::Time(0));
  			global_transform = transformStamped.transform;
  		}
  		catch(tf2::TransformException &ex){
  			ROS_WARN("from main %s",ex.what());
  			continue;
  		}
  			
  		//clearOccupancyGrid();	
		fillOccupancyGrid();
		gridpub.publish(*occupancy_grid);
	}
}
