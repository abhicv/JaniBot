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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <math.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <bits/stdc++.h>
using namespace std;

//below dimensions are for 90ml to 200ml range 
//cup base radius range:2-3
//cup top radius range:2.5-3.75

void passthrough_filter(pcl::PointCloud<pcl::PointNormal>::Ptr &cld_in,pcl::PointCloud<pcl::PointNormal>::Ptr &cld_out,double filter_limit,std::string& dir,bool b)
{
pcl::PassThrough<pcl::PointNormal> pass;
pass.setInputCloud (cld_in);
pass.setFilterFieldName(dir);
pass.setFilterLimits (0.0, filter_limit);
pass.setFilterLimitsNegative (b);
pass.filter (*cld_out);
}


void pre_processing(pcl::PointCloud<pcl::PointNormal>::Ptr& cld, float& obj_max_height,float& pre_processing_dist)
{
float x,y,z,dist;
string dir="z";
bool filter_negative=false;
passthrough_filter(cld,cld,obj_max_height,dir,filter_negative);
for (int i=0; i<cld->size(); i++)
{
x=cld->points[i].x;
y=cld->points[i].y;
z=cld->points[i].z;
dist= pow(x*x + y*y + z*z,0.5);
if (dist <= pre_processing_dist)
{
cld->points[i].x=0.01;
cld->points[i].y=0;
cld->points[i].z=0;
}
}
dir="x";
filter_negative=true;
passthrough_filter(cld,cld,0.02,dir,filter_negative);
}


void voxel_filter(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,float& leaf_size)
{
pcl::VoxelGrid<pcl::PointNormal> fltr;
fltr.setInputCloud(cld);
fltr.setLeafSize(leaf_size,leaf_size,leaf_size);
fltr.filter(*cld);
}


void remove_ground_plane(pcl::PointCloud<pcl::PointNormal>::Ptr& cld,float& groundplane_dist_thresh)
{
pcl::PointCloud<pcl::PointNormal>::Ptr temp (new pcl::PointCloud<pcl::PointNormal>);
std::string dir="z";
bool filter_negative=false;
passthrough_filter(cld,temp,groundplane_dist_thresh,dir,filter_negative);
if (temp->size()!=cld->size())
{
filter_negative=true;
passthrough_filter(cld,cld,groundplane_dist_thresh,dir,filter_negative);
}
else
{
cld->clear();
}
}


void compute_normal(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,float& radius_search)
{
if(!cld->empty())
{
pcl::NormalEstimation<pcl::PointNormal,pcl::PointNormal> ne;
ne.setInputCloud(cld);
pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>() );
ne.setSearchMethod(tree);
ne.setRadiusSearch(radius_search);
ne.compute(*cld);
pcl::io::savePCDFileASCII("pre_processing.pcd",*cld);
}
}


void convert(pcl::PointCloud<pcl::PointNormal>::Ptr &nwpt,pcl::PointCloud<pcl::Normal>::Ptr &normal)
{
for (std::size_t i=0;i<nwpt->size();i++)
{
pcl::Normal nr;
nr.normal_x=nwpt->points[i].normal_x;
nr.normal_y=nwpt->points[i].normal_y;
nr.normal_z=nwpt->points[i].normal_z;
nr.curvature=nwpt->points[i].curvature;
normal->points.push_back(nr);
}
}


int ransac_plan_segmentation(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,double dist_threshold)
{
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
pcl::SACSegmentation<pcl::PointNormal> seg;
seg.setOptimizeCoefficients (true);
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setDistanceThreshold (dist_threshold);
seg.setInputCloud (cld);
seg.segment (*inliers, *coefficients);
return inliers->indices.size();
}


void region_growing_segmentation(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,float& angle_limit,float& curvature_limit,float& minclus_size,float& maxclus_size,float& neighbours,float& threshold,float& plane_tolerance)
{
if(!cld->empty())
{
int counter=0;
pcl::RegionGrowing<pcl::PointNormal, pcl::Normal> reg;
pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
pcl::search::Search<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
convert(cld,normal);
reg.setMinClusterSize (int(minclus_size));
reg.setMaxClusterSize (int(maxclus_size));
reg.setSearchMethod (tree);
reg.setNumberOfNeighbours (int(neighbours));
reg.setInputCloud (cld);
reg.setInputNormals (normal);
reg.setSmoothnessThreshold ((angle_limit*M_PI)/180.0);
reg.setCurvatureThreshold (curvature_limit);
vector<pcl::PointIndices> clusters;
reg.extract (clusters);
int size;
int pos;
for(unsigned int i=0;i<clusters.size();i++)
{
pcl::PointCloud<pcl::PointNormal>::Ptr temp (new pcl::PointCloud<pcl::PointNormal>);
pcl::copyPointCloud(*cld,clusters[i],*temp);
size=ransac_plan_segmentation(temp,threshold);
if (size >= int(plane_tolerance*temp->size()) )
{
for (int j=0; j<clusters[i].indices.size(); j++)
{
pos=clusters[i].indices[j];
cld->points[pos].x=0.01;
cld->points[pos].y=0;
cld->points[pos].z=0;
counter++;
}
}
}
if(counter < cld->size())
{
string dir="x";
bool filter_negative=true;
passthrough_filter(cld,cld,0.02,dir,filter_negative);
pcl::io::savePCDFileASCII("planes_removed.pcd",*cld);
}
else
{
cld->clear();
}
}
}


int find_line(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,float edge_threshold)
{
std::vector<int> inliers;
pcl::SampleConsensusModelLine<pcl::PointNormal>::Ptr line
     (new pcl::SampleConsensusModelLine<pcl::PointNormal> (cld));
pcl::RandomSampleConsensus<pcl::PointNormal> ransac(line);
ransac.setDistanceThreshold(edge_threshold);
ransac.computeModel();
ransac.getInliers(inliers);
return inliers.size();
}


vector<pcl::PointIndices> remove_edges(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,float& cluster_threshold,float& edge_threshold,float& min_cluster_size, float& max_cluster_size,float& edge_tolerance)
{
std::vector<pcl::PointIndices> chosen_clusters;
if(!cld->empty())
{
pcl::PointCloud<pcl::PointNormal>::Ptr disp (new pcl::PointCloud<pcl::PointNormal>);
pcl::copyPointCloud(*cld,*disp);
pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
tree->setInputCloud (cld);
vector<pcl::PointIndices> clusters;
pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
ec.setClusterTolerance (cluster_threshold); 
ec.setMinClusterSize (int(min_cluster_size));
ec.setMaxClusterSize (int(max_cluster_size));
ec.setSearchMethod (tree);
ec.setInputCloud (cld);
ec.extract (clusters);
unsigned int pos;
int size;
for(unsigned int i=0;i<clusters.size();i++)
{
pcl::PointCloud<pcl::PointNormal>::Ptr temp (new pcl::PointCloud<pcl::PointNormal>);
pcl::copyPointCloud(*cld,clusters[i],*temp);
size=find_line(temp,edge_threshold);
if (size <= int( edge_tolerance*temp->size() ))
{
chosen_clusters.push_back(clusters[i]);
}
else
{
for (int j=0; j<clusters[i].indices.size(); j++)
{
pos=clusters[i].indices[j];
disp->points[pos].x=0.01;
disp->points[pos].y=0.0;
disp->points[pos].z=0.0;
}
}
}
string dir="x";
bool filter_negative=true;
passthrough_filter(disp,disp,0.02,dir,filter_negative);
pcl::io::savePCDFileASCII("edges_removed.pcd",*disp);
}
return chosen_clusters;
}


vector<pcl::ModelCoefficients> trash_segmentation(pcl::PointCloud<pcl::PointNormal>::Ptr &cld,std::vector<pcl::PointIndices> &clusters_in,std::vector<pcl::PointIndices> &clusters_out,float& min_rad,float& max_rad)
{
vector<pcl::ModelCoefficients> trash_coefficients;
if(!clusters_in.empty())
{
int counter=0;
pcl::SACSegmentationFromNormals<pcl::PointNormal,pcl::PointNormal> seg;
seg.setOptimizeCoefficients (true);
seg.setModelType (pcl::SACMODEL_CYLINDER);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setNormalDistanceWeight (0.1);
seg.setMaxIterations (1000);
seg.setDistanceThreshold (0.05);
seg.setRadiusLimits (0,1);
for(int i=0;i<clusters_in.size();i++)
{
pcl::PointCloud<pcl::PointNormal>::Ptr temp (new pcl::PointCloud<pcl::PointNormal>);
pcl::copyPointCloud(*cld,clusters_in[i],*temp);
seg.setInputCloud (temp);
seg.setInputNormals (temp);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
seg.segment (*inliers, *coefficients);
float rad=coefficients->values[6];
if (inliers->indices.size()>0)
{
if(rad>min_rad && rad<max_rad)
{ 
trash_coefficients.push_back(*coefficients);
clusters_out.push_back(clusters_in[i]);
string name="trash_extracted";
string number=to_string(counter);
name.append(number);
string extension=".pcd";
name.append(extension);
pcl::io::savePCDFileASCII(name,*temp);
counter++;
}
}
}
}
return trash_coefficients;
}

class listener
{

public:
pcl::PointCloud<pcl::PointNormal>::Ptr cld;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
geometry_msgs::TransformStamped transform_value;
pcl::PointCloud<pcl::PointNormal> cloud_in;

pcl::fromROSMsg(*input,cloud_in);
ros::Duration(1.0).sleep();
transform_value=tfBuffer.lookupTransform("base_frame","camera_depth_optical_frame",ros::Time(0));
double y,p,r;
tf2::Quaternion quat(transform_value.transform.rotation.x,transform_value.transform.rotation.y,transform_value.transform.rotation.z,transform_value.transform.rotation.w);
tf2::Matrix3x3(quat).getRPY(r,p,y);
Eigen::Transform<tf2Scalar,3,Eigen::Affine> transform;
pcl::getTransformation(transform_value.transform.translation.x,transform_value.transform.translation.y,transform_value.transform.translation.z,r,p,y,transform);
pcl::transformPointCloud(cloud_in,*cld,transform);
}
};

