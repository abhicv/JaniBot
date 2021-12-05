#include "object_extraction.h"
listener lis;
int main(int argc,char **argv)
{
ros::init(argc,argv,"main");
ros::NodeHandle nh;
pcl::PointCloud<pcl::PointNormal>::Ptr nwpt (new pcl::PointCloud<pcl::PointNormal>);

lis.cld=nwpt;
ros::Subscriber sub=nh.subscribe("/camera/depth/points", 10 , &listener::callback, &lis);
ros::Rate r(70);
r.sleep();

while(ros::ok()){
ros::spinOnce();
float leaf_size=0.007;
float radius_search=0.01;
float angle_limit=3; //in degrees
float curvature_limit=0.1;
float minclus_size_rg=30;
float maxclus_size_rg=1000000;
float neighbours=5;
float cylmax_rad=0.05;
float cylmin_rad=0.02;
float cyl_rad;
float obj_max_height=0.15;
float edge_threshold=0.02;
float groundplane_dist_thresh=0.01;
float cluster_threshold=0.02;
float minclus_size_ce=30;
float maxclus_size_ce=1000000;
float edge_tolerance= 0.8;
float plane_threshold=0.0001;
float plane_tolerance= 0.9;
float pre_processing_dist=0.6;

std::vector<pcl::PointIndices> clusters_in;
std::vector<pcl::PointIndices> clusters_out;
vector<pcl::ModelCoefficients> coefficients;

pre_processing(nwpt,obj_max_height,pre_processing_dist);
voxel_filter(nwpt,leaf_size);
remove_ground_plane(nwpt,groundplane_dist_thresh);
compute_normal(nwpt,radius_search);
region_growing_segmentation(nwpt,angle_limit,curvature_limit,minclus_size_rg,maxclus_size_rg,neighbours,plane_threshold,plane_tolerance);
clusters_in=remove_edges(nwpt,cluster_threshold,edge_threshold,minclus_size_ce, maxclus_size_ce, edge_tolerance);
coefficients=trash_segmentation(nwpt,clusters_in,clusters_out,cylmin_rad,cylmax_rad);
cout<<endl<<coefficients[0].values[0];
r.sleep();
}
}

