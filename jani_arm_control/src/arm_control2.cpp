#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

//#include <tf2/transform_datatypes.h>
//#include <LinearMath/btMatrix3x3.h>

//#include <tf2/Matrix3x3.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>

geometry_msgs::TransformStamped transform_value;
void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
pcl::PointCloud<pcl::PointXYZ> cloud_in;
pcl::PointCloud<pcl::PointXYZ> cloud_out;
pcl::PCLPointCloud2 *cloud= new pcl::PCLPointCloud2;
pcl::PCLPointCloud2::ConstPtr cloudPtr(cloud);
pcl_conversions::toPCL(*input, *cloud);
pcl::fromROSMsg(*input,cloud_in);
//pcl::io::savePCDFileASCII("pcd_test.pcd",cloud_temp);
ROS_INFO("fucku");
transform_value=tfBuffer.lookupTransform("camera_link","camera_depth_optical_frame",ros::Time(0));
double y,p,r;
tf2::Quaternion quat(transform_value.transform.rotation.x,transform_value.transform.rotation.y,transform_value.transform.rotation.z,transform_value.transform.rotation.w);
tf2::Matrix3x3(quat).getRPY(r,p,y);
//m.setRPY(r,p,y);
ROS_INFO("The transform quaternion of transform is %.001f,%.001f,%.001f ",r,p,y);
Eigen::Transform<tf2Scalar,3,Eigen::Affine> transform;
pcl::getTransformation(0,0,0,r,p,y,transform);
pcl::transformPointCloud(cloud_in,cloud_out,transform);
pcl::io::savePCDFileASCII("pcd_test1.pcd",cloud_out);

ros::shutdown();
}

int main(int argc,char **argv)
{
ros::init(argc,argv,"arm_control2");
ros::NodeHandle nh;

ros::Subscriber sub=nh.subscribe("/camera/depth/points",10,callback);
ros::spin();

}

