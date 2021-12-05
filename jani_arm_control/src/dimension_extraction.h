#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <pcl/impl/point_types.hpp>
using namespace std;
float* extract_centre_height(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, pcl::PointIndices &cluster, float axis_points[3], float axis_dir[3] )
{

float temp[3];
vector<float> hpt;
int flag=0;
for (for int i=0;i<cluster.indices.size();i++)
{
int point=cluster.indices[i];
float px=cld->points[point].x;
float py=cld->points[point].y;
float pz=cld->points[point].z;

float k= (axis_dir[0]*(px-axis_points[0])+axis_dir[1]*(py-axis_points[1])+axis_dir[2]*(pz-axis_points[2]))/(axis_dir[0]*axis_dir[0]+axis_dir[1]*axis_dir[1]+axis_dir[2]*axis_dir[2]);

for (int i=0;i<3;i++)
{
temp[i]=axis_points[i]+axis_dir[i]*k;
}

if (flag<=1)
{
hpt.push_back(temp[0]);
hpt.push_back(temp[1]);
hpt.push_back(temp[2]);
++flag;

}
else
{

float dist_curr=pow(pow(hpt[0]-hpt[3],2)+pow(hpt[1]-hpt[4],2)+pow(hpt[2]-hpt[5],2),0.5);
float d1=pow(pow(hpt[0]-temp[0],2)+pow(hpt[1]-temp[1],2)+pow(hpt[2]-temp[2],2),0.5);
float d2=pow(pow(hpt[3]-temp[0],2)+pow(hpt[4]-temp[1],2)+pow(hpt[5]-temp[2],2),0.5);
if (d1+d2> dist_curr)
{
if (d1<d2)
{
hpt[0]=temp[0];
hpt[1]=temp[1];
hpt[2]=temp[2];

}
else if (d2<d1)
{
hpt[3]=temp[0];
hpt[4]=temp[1];
hpt[5]=temp[2];

}
else if (d1==d2)
{
hpt[0]=temp[0];
hpt[1]=temp[1];
hpt[2]=temp[2];

}
}
}
}

static float centre_height[4];
centre_height[0]=(hpt[0]+hpt[3])/2.0;
centre_height[1]=(hpt[1]+hpt[4])/2.0;
centre_height[2]=(hpt[2]+hpt[5])/2.0;
centre_height[3]=pow(pow(hpt[0]-hpt[3],2)+pow(hpt[1]-hpt[4],2)+pow(hpt[2]-hpt[5],2),0.5);
return centre_height;
}


float extract_radius(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud,pcl::PointIndices &cluster, float point[3], float axis_dir[3], float radius_angle_thresh)
{
for (int i=0; i<cluster.indices.size(); i++)
{
int point=cluster.indices[i];
float px=cld->points[point].x;
float py=cld->points[point].y;
float pz=cld->points[point].z;
float v1[3];
float radius;
v1[0]=(px-point[0])/pow(pow(px-point[0],2)+pow(py-point[1],2)+pow(pz-point[2],2),0.5);
v1[1]=(py-point[1])/pow(pow(px-point[0],2)+pow(py-point[1],2)+pow(pz-point[2],2),0.5);
v1[2]=(pz-point[2])/pow(pow(px-point[0],2)+pow(py-point[1],2)+pow(pz-point[2],2),0.5);
axis_dir[0]=axis_dir[0]/pow(pow(axis_dir[0],2)+pow(axis_dir[1],2)+pow(axis_dir[2],2),0.5);
axis_dir[1]=axis_dir[0]/pow(pow(axis_dir[0],2)+pow(axis_dir[1],2)+pow(axis_dir[2],2),0.5);
axis_dir[2]=axis_dir[0]/pow(pow(axis_dir[0],2)+pow(axis_dir[1],2)+pow(axis_dir[2],2),0.5);
float min=cos(M_PI/2 + radius_angle_thresh);
float max=cos(M_PI/2 - radius_angle_thresh);
if (v1[0]*axis_dir[0]+ v1[1]*axis_dir[1] + v1[2]*axis_dir[2]<max && v1[0]*axis_dir[0]+ v1[1]*axis_dir[1] + v1[2]*axis_dir[2]>min)
{
radius=pow(pow(px-point[0],2)+pow(py-point[1],2)+pow(pz-point[2],2),0.5);
break;
}
}
return radius;
}


