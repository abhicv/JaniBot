//ros headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

//pcl headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/geometry.h>


//for visualizing in rviz
pcl::PointCloud<pcl::PointXYZ> final;
geometry_msgs::PolygonStamped door;
geometry_msgs::PointStamped min_p, max_p;

//NOTE(ahbicv): Now separate function for wall segmentation and door detection
struct SegmentedWallPlane
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
};

//given a point cloud data return array of segmented wall planes
std::vector<SegmentedWallPlane>
SegmentWallPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg)
{
    std::vector<SegmentedWallPlane> wall_planes;
    
    //downsampled cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    
    //voxel filtering for down sample input point cloud to speed up segmentation
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_msg);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_downsampled);
    
    //looping until no more wall plane is found
    while(1)
    {  
        //wall plane cloud
        pcl::PointCloud<pcl::PointXYZ> cloud_plane;
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        
        // Segmenting a wall plane(parallel to yaxis)
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setMaxIterations(50);
        seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0)); //axiz the plane is to be parallel(here y axiz)
        seg.setEpsAngle((2.0 * M_PI) / 180.0);
        seg.setInputCloud(cloud_downsampled);
        seg.segment (*inliers, *coefficients);
        
        if(inliers->indices.size() > 0)
        {
            //extract wall plane point cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_downsampled);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter(cloud_plane);
            
            //extract original point cloud minus detected wall plane
            pcl::ExtractIndices<pcl::PointXYZ> extract_orginal;
            extract_orginal.setInputCloud (cloud_downsampled);
            extract_orginal.setIndices (inliers);
            extract_orginal.setNegative (true);
            extract_orginal.filter(*cloud_downsampled);
            
            SegmentedWallPlane wall_plane;
            wall_plane.cloud = cloud_plane;
            wall_plane.inliers = inliers;
            wall_plane.coefficients = coefficients;
            
            wall_planes.push_back(wall_plane);
        }
        else
        {
            break;
        }
    }
    
    return wall_planes;
}

//given a wall plane detects door and plot a door polygon in rviz via polygon geometry msg
void DetectDoorInWallPlane(SegmentedWallPlane &wall_plane)
{
    if (wall_plane.inliers->indices.size() > 0)
    {
        //getting plane normal
        pcl::PointXYZ normal;
        normal.x = wall_plane.coefficients->values[0];
        normal.y = wall_plane.coefficients->values[1];
        normal.z = wall_plane.coefficients->values[2];
        //ROS_INFO("normal: %f %f %f", normal.x, normal.y, normal.z);
        
        // getting a strip from cloud plane at a specific height
        pcl::PointCloud<pcl::PointXYZ>::Ptr strip(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> indices;
        for(int n = 0; n < wall_plane.cloud.size(); n++)
        {
            if(wall_plane.cloud.points[n].y < 0.03 && wall_plane.cloud.points[n].y > -0.03)
            {
                indices.push_back(n);
            }
        }
        
        if(indices.size() > 0)
        {
            pcl::copyPointCloud(wall_plane.cloud, indices, final);
            pcl::copyPointCloud(wall_plane.cloud, indices, *strip);
            
            //finding points in strip cloud with min and max x values 
            pcl::PointXYZ min_pt, max_pt;
            min_pt = strip->points[0];
            max_pt = strip->points[0];
            
            for(int n = 0; n < strip->size(); n++)
            {
                if(strip->points[n].x < min_pt.x)
                {
                    min_pt = strip->points[n];
                }
                if(strip->points[n].x > max_pt.x)
                {
                    max_pt = strip->points[n];
                }
            }
            
            min_p.point.x = min_pt.x; min_p.point.y = min_pt.y; min_p.point.z = min_pt.z;
            max_p.point.x = max_pt.x; max_p.point.y = max_pt.y; max_p.point.z = max_pt.z;
            
            float dist = pcl::geometry::distance(min_pt, max_pt);
            
            pcl::PointXYZ dir;
            dir.x = max_pt.x - min_pt.x;
            dir.y = max_pt.y - min_pt.y;
            dir.z = max_pt.z - min_pt.z;
            dir.x /= dist;
            dir.y /= dist;
            dir.z /= dist;
            
            float search_radius = 0.05;
            int n_prev = 1;
            
            //stting up kd tree for a radial search of neighbouring points
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(strip);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            
            //door.polygon.points.clear();
            
            //searching for gap in strip cloud
            pcl::PointXYZ pc;
            while(pcl::geometry::distance(min_pt, max_pt) > 0.1)
            {
                int n_curr = kdtree.radiusSearch(min_pt, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                
                //ROS_INFO("min pt : %f, %f, %f, max pt : %f, %f, %f", min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);
                //ROS_INFO("n_curr: %d, dist: %f", n_curr, pcl::geometry::distance(min_pt, max_pt));
                
                if(n_curr == 0 && n_prev > 0)
                {
                    pc = min_pt;
                }
                else if(n_curr > 0 && n_prev == 0)
                {
                    pcl::PointXYZ pe = min_pt;
                    
                    float gap = pcl::geometry::distance(pc, pe);
                    ROS_INFO("gap found! gap size : %f", gap);
                    
                    //min door width known 0.8 meters
                    if(gap > 0.8)
                    {
                        //NOTE(abhicv): creating the door polygon for viz
                        geometry_msgs::Point32 p;
                        
                        //top left
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
                
                //moving 10 cm on each iteration towards the max point along the strip from min point
                min_pt.x += dir.x / 100;
                min_pt.y += dir.y / 100;
                min_pt.z += dir.z / 100;
            }
        }
    }
}

#if 0
void DetectDoor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg)
{
    //downsampled cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    //do a voxel filtering for down sample input point cloud to speed up segmentation
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_msg);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    
    //wall plane cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
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
        //pcl::copyPointCloud(*cloud_filtered, inliers->indices, final);
        
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
            if(cloud_plane->points[n].y < 0.02 && cloud_plane->points[n].y > -0.02)
            {
                indices.push_back(n);
            }
        }
        
        if(indices.size() > 0)
        {
            pcl::copyPointCloud(*cloud_plane, indices, final);
            pcl::copyPointCloud(*cloud_plane, indices, *strip);
            
            //finding points in strip cloud with min and max x values 
            pcl::PointXYZ min_pt, max_pt;
            min_pt = strip->points[0];
            max_pt = strip->points[0];
            
            for(int n = 0; n < strip->size(); n++)
            {
                if(strip->points[n].x < min_pt.x)
                {
                    min_pt = strip->points[n];
                }
                if(strip->points[n].x > max_pt.x)
                {
                    max_pt = strip->points[n];
                }
            }
            
            min_p.point.x = min_pt.x; min_p.point.y = min_pt.y; min_p.point.z = min_pt.z;
            max_p.point.x = max_pt.x; max_p.point.y = max_pt.y; max_p.point.z = max_pt.z;
            
            float dist = pcl::geometry::distance(min_pt, max_pt);
            
            pcl::PointXYZ dir;
            dir.x = max_pt.x - min_pt.x;
            dir.y = max_pt.y - min_pt.y;
            dir.z = max_pt.z - min_pt.z;
            dir.x /= dist;
            dir.y /= dist;
            dir.z /= dist;
            
            float search_radius = 0.05;
            int n_prev = 1;
            
            //stting up kd tree for a radial search of neighbouring points
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(strip);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            
            pcl::PointXYZ pc;
            
            door.polygon.points.clear();
            
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
                        
                        //TODO(abhicv): to find position and orientation of the door      
                        break;
                    }
                }
                n_prev = n_curr;
                
                //moving 10 cm on each iteration towards the max point along the strip from min point
                min_pt.x += dir.x / 100;
                min_pt.y += dir.y / 100;
                min_pt.z += dir.z / 100;
            }
        }
    }
}
#endif

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{ 
    ROS_INFO("Point cloud data recevied!!\n");
    //raw cloud point as received through msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_msg);

    door.polygon.points.clear();
    // DetectDoor(cloud_msg);

    //iterating through all segmented walls
    std::vector<SegmentedWallPlane> wall_planes = SegmentWallPlanes(cloud_msg);
    for(int n = 0; n < wall_planes.size(); n++)
    {
        DetectDoorInWallPlane(wall_planes[n]);
    }
}

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "DoorDetector");
    
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe("/camera/depth/points", 1, PointCloudCallback);
    
    std::string frame("camera_depth_optical_frame");
    
    //publishing segmented wall cloud point for rviz
    ros::Publisher pub = nh.advertise<PointCloud> ("/wall_cloud", 1);
    final.header.frame_id = frame;
    
    //publishing door polygon for rviz
    ros::Publisher poly_pub = nh.advertise<geometry_msgs::PolygonStamped> ("/polygon", 5);
    door.header.frame_id = frame;
    
    //publishing min and max points
    ros::Publisher min_p_pub = nh.advertise<geometry_msgs::PointStamped> ("/min_point", 5);
    min_p.header.frame_id = frame;
    ros::Publisher max_p_pub = nh.advertise<geometry_msgs::PointStamped> ("/max_point", 5);
    max_p.header.frame_id = frame;
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    ros::Rate loop_rate(10);
    
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), final.header.stamp);
        
        pub.publish (final);
        poly_pub.publish (door);
        
        //NOTE(abhicv): ignore this! testing tf transforms
        // try{
        //    listener.lookupTransform("camera_color_optical_frame", "odom_frame", ros::Time(0), transform);
        // }
        // catch (tf::TransformException &ex) {
        //   ROS_ERROR("%s",ex.what());
        //   ros::Duration(1.0).sleep();
        //   continue;
        // }
        // geometry_msgs::PointStamped min_p_, max_p_;
        // min_p_.header.frame_id = "odom_frame";
        // max_p_.header.frame_id = "odom_frame";
        
        // try{
        //     listener.transformPoint("odom_frame", min_p, min_p_);
        // }
        // catch (tf::TransformException &ex) {
        //     ROS_ERROR("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }
        
        // try{
        //     listener.transformPoint("odom_frame", max_p, max_p_);
        // }
        // catch (tf::TransformException &ex) {
        //     ROS_ERROR("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }

        min_p_pub.publish(min_p);
        max_p_pub.publish(max_p);
        
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    return 0;
}