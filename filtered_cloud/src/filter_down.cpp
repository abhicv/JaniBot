#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>


#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>





ros::Publisher pub;
tf2_ros::Buffer tfbuffer;


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::fromROSMsg(*cloud_msg,*cloud);
    const std::string name = "base_link";
	//const pcl::PointCloud<pcl::PointXYZ>::Ptr test(cloud);
	pcl_ros::transformPointCloud(name,*cloud,*cloud_transformed,tfbuffer);

    sor.setInputCloud (cloud_transformed);
  	sor.setLeafSize(0.1,0.1,0.1);
  	sor.filter(*cloud_filtered);

    std::cout<<"Hello World";

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);
    // Publish the data.
  	pub.publish (output);

	}










int main(int argc, char** argv){
  	ros::init (argc, argv, "passthrough");
  	ros::NodeHandle nh;
	tf2_ros::TransformListener tfListener(tfbuffer);

  	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, pointCloudCallback);

  	pub = nh.advertise<sensor_msgs::PointCloud2> ("processed_cloud", 1);
  	while (ros::ok())
  	{


  		geometry_msgs::TransformStamped transformStamped;
  		try{
  			transformStamped = tfbuffer.lookupTransform("camera_depth_optical_frame","base_link",ros::Time(0));
  		}
  		catch(tf2::TransformException &ex){
  			ROS_WARN("%s",ex.what());
  			continue;
  		}

  		ros::spinOnce();
	}
}
