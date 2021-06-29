
#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/point_cloud_color_handlers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>

pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("pcl viewer"));
pcl::PCLPointCloud2 *cloud= new pcl::PCLPointCloud2;
pcl::PCLPointCloud2::ConstPtr cloudPtr(cloud);

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
pcl_conversions::toPCL(*input, *cloud);
ROS_INFO("fucku");
viewer->setBackgroundColor(0,0,0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>::ConstPtr bla (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>(cloudPtr,255.0,0.0,0.0));
Eigen::Vector4f v;
Eigen::Quaternion<float> q;
viewer->addPointCloud(cloudPtr,bla,v,q);
}

int main(int argc,char **argv)
{
ros::init(argc,argv,"arm_control2");
ros::NodeHandle nh;
ROS_INFO("The cloud is bla");
ros::Rate freq(39);
ros::Subscriber sub=nh.subscribe("/camera/depth/points",10,callback);
while (!viewer->wasStopped())
  { 
    ros::spinOnce();
    viewer->spinOnce(100);
    freq.sleep();
  }  

}

