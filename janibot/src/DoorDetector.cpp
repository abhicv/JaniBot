#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/geometry.h>

int first = 1;

pcl::PointCloud<pcl::PointXYZ> final;
geometry_msgs::PolygonStamped door;

// using pcl sample consensus module
#if 0
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pc2_msg)
{ 
  ROS_INFO("Point cloud data recevied!!\n");

  if(first)
  {
    first = 1;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc2_msg ,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    
    pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr 
                  plane_model(new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>(temp_cloud));
    plane_model->setAxis(Eigen::Vector3f(0.0, 1.0, 0.0f));
    plane_model->setEpsAngle(2.0 * 3.14 / 180.0);

    std::vector<int> inliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
    ransac.setDistanceThreshold (0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud(*temp_cloud, inliers, final);
    
    if(inliers.size() > 0)
    {
      ROS_INFO("inlier size: %d\n", inliers.size());
    }  
  }
}
#endif

//NOTE(abhicv): using the pcl segmentation module
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{ 
  ROS_INFO("Point cloud data recevied!!\n");

  //raw cloud point as received through msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_msg);

  //downsampled cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
  //wall plane cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  
  //do a voxel filtering for down sampling to speed up segmentation
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_msg);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  //NOTE(abhicv): camera frame
  // z axiz forward
  // x axis side ways
  // y axis downwards perpedicular to the ground

  // Segmenting a wall plane(parallel to yaxis)
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setMaxIterations(50);

  //axiz the plane is to be parallel(here y axiz)
  seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
  seg.setEpsAngle((2.0 * M_PI) / 180.0);
  seg.setInputCloud(cloud_filtered);
  seg.segment (*inliers, *coefficients);
   
  if (inliers->indices.size() > 0)
  {
    ROS_INFO("Found wall plane!!, max iter:%d", seg.getMaxIterations());
    pcl::copyPointCloud(*cloud_filtered, inliers->indices, final);

    //extraxting plane cloud points from downsampled cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*cloud_plane);
    
    //getting plane normal
    pcl::PointXYZ normal;
    normal.x = coefficients->values[0];
    normal.y = coefficients->values[1];
    normal.z = coefficients->values[2];

    ROS_INFO("normal: %f %f %f", normal.x, normal.y, normal.z);

    // getting a strip from cloud plane at a specific height
    pcl::PointCloud<pcl::PointXYZ>::Ptr strip(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    for(int n = 0; n < cloud_plane->size(); n++)
    {
        if(cloud_plane->points[n].y < 0.03 && cloud_plane->points[n].y > -0.03)
        {
          indices.push_back(n);
          //ROS_INFO("point %d : %f, %f, %f\n", n, cloud_plane->points[n].x, cloud_plane->points[n].y, cloud_plane->points[n].z);
        }
    }
    //pcl::copyPointCloud(*cloud_plane, indices, final);
    pcl::copyPointCloud(*cloud_plane, indices, *strip);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*strip, min_pt, max_pt);
    
    float search_radius = 0.05;
    int n_prev = 1;
    
    //stting up kd tree for a radial search of neighbouring points
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(strip);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointXYZ pc;

    door.polygon.points.clear();

#if 1

    //searching for gap in strip cloud
    while(pcl::geometry::distance(min_pt, max_pt) > 0.1)
    {
      int n_curr = kdtree.radiusSearch(min_pt, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      ROS_INFO("min pt : %f, %f, %f, max pt : %f, %f, %f", min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);
      ROS_INFO("n_curr: %d, dist: %f", n_curr, pcl::geometry::distance(min_pt, max_pt));

      if(n_curr == 0 && n_prev > 0)
      {
        pc = min_pt;
      }
      else if(n_curr > 0 && n_prev == 0)
      {
        pcl::PointXYZ pe = min_pt;
        
        float gap = pcl::geometry::distance(pc, pe);
        ROS_INFO("gap found! gap size : %f", gap);

        if(gap > 0.8)
        {
          //NOTE(abhicv): creating the door polygon for viz

          //top left
          geometry_msgs::Point32 p;
          p.x = pc.x;
          p.y = -0.6;
          p.z = pc.z; 
          door.polygon.points.push_back(p);
          
          //top right
          p.x = pe.x;
          p.y = -0.6;
          p.z = pe.z; 
          door.polygon.points.push_back(p);

          //bottom right
          p.x = pe.x;
          p.y = 1.4;
          p.z = pe.z; 
          door.polygon.points.push_back(p);

          //bottom left
          p.x = pc.x;
          p.y = 1.4;
          p.z = pc.z; 
          door.polygon.points.push_back(p);

          break;
        }
      }
      n_prev = n_curr;

      min_pt.x += normal.z / 100;
      min_pt.y += 0;
      min_pt.z += -normal.x / 100;
    }
#endif
  }
}

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "DoorDetector");std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub = nh.subscribe("/camera/depth/points", 1, PointCloudCallback);

  //publishing segmented wall cloud point for rviz
  ros::Publisher pub = nh.advertise<PointCloud> ("/points2", 5);

  //publishing door polygon for rviz
  ros::Publisher poly_pub = nh.advertise<geometry_msgs::PolygonStamped> ("/polygon", 5);
  std::string frame("camera_color_optical_frame");
  door.header.frame_id = frame;

  ros::Rate loop_rate(5);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), final.header.stamp);
    pub.publish (final);
    poly_pub.publish(door);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}