#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/impl/search.hpp>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <visualization_msgs/Marker.h>
using namespace std;
geometry_msgs::Point center_object;

class PointCloudProc
{
  public:
    PointCloudProc() : cloud_transformed_(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
                       cloud_hull(new pcl::PointCloud<pcl::PointXYZ>), cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudProc::pointcloudcb, this);
        point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed_point_cloud", 1000);
        seg_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("segmented_plane_point_cloud", 1000);
        filtered_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_point_cloud", 1000);
        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        fixed_frame_ = "/root";
    }

    void pointcloudcb(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
    {
        geometry_msgs::Point plane_normal;
        pcl::ExtractIndices<pcl::PointXYZ> extract_;
        pcl::ConvexHull<pcl::PointXYZ> chull;
        cloud_raw_ = msg;

        bool transform_success = pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed_, listener_);
        point_cloud_pub_.publish(cloud_transformed_);

        if (transform_success == 0) 
        {
          ROS_INFO("failed transform point cloud");
          return;
        }

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_transformed_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(center_object.x - 0.1, center_object.x + 0.5);
        pass.filter(*cloud_filtered_);
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(center_object.y - 0.6, center_object.y + 0.0);
        pass.filter(*cloud_filtered_);
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(center_object.z - 0.0, center_object.z + 0.7);
        pass.filter(*cloud_filtered_);

        ROS_INFO("Point cloud is filtered!");
        std::cout << cloud_filtered_->points.size() << " of points in the filtered point cloud" << std::endl;
        if (cloud_filtered_->points.size() == 0)
        {
          ROS_INFO("Point cloud is empty after filtering!");
        }

        // publish filtered point cloud
        // filtered_point_cloud_pub_.publish(cloud_filtered_);
        
        // planar segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0); //z axis
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory set plane to be parallel to Z axis within a 15 degrees tolerance
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMaxIterations(500); // iteration limits decides segmentation goodness
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(15.0f));
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud_transformed_);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
          {
              PCL_ERROR("Could not estimate a planar model for the given dataset.");
              return;
          }

        // extract inline points original point coulds
        extract_.setInputCloud(cloud_transformed_);
        extract_.setNegative(false);
        extract_.setIndices(inliers);
        extract_.filter(*cloud_plane);
        ROS_INFO_STREAM("# of points in plane: " << cloud_plane->points.size());

        // publish segmented point cloud
        seg_point_cloud_pub_.publish(cloud_plane);

        // Create a Convex Hull representation of the plane
        chull.setInputCloud(cloud_plane);
        chull.setDimension(2);
        chull.reconstruct(*cloud_hull);

        // Get plane center
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cloud_plane, center);
        std::cout << "center: " << center[0] << "," << center[1] << "," << center[2] << std::endl;

        // Get plane min and max values
        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cloud_plane, min_vals, max_vals); 

        // Get plane polygon
        for (int i = 0; i < cloud_hull->points.size(); i++)
        {
            geometry_msgs::Point32 p;
            p.x = cloud_hull->points[i].x;
            p.y = cloud_hull->points[i].y;
            p.z = cloud_hull->points[i].z;
        }  


        // Get plane normal
        float length = sqrt(coefficients->values[0] * coefficients->values[0] +
                            coefficients->values[1] * coefficients->values[1] +
                            coefficients->values[2] * coefficients->values[2]);
        plane_normal.x = coefficients->values[0] / length;
        plane_normal.y = coefficients->values[1] / length;
        plane_normal.z = coefficients->values[2] / length;
        std::cout << "normal: " << plane_normal.x << "," << plane_normal.y << "," << plane_normal.z << std::endl;
    }
    

  private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher plane_pub_;
    ros::Publisher point_cloud_pub_, seg_point_cloud_pub_, filtered_point_cloud_pub_;
    ros::Publisher marker_pub;
    ros::ServiceServer planar_segment_src_;
    std::string fixed_frame_;

    tf::TransformListener listener_;
    tf::StampedTransform transform;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_, cloud_transformed_, cloud_filtered_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
    pcl::ConvexHull<pcl::PointXYZ> chull;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_plane_segmentation");

    center_object.x = 0;
    center_object.y = 0;
    center_object.z = 0;
    PointCloudProc pc_tools;
    ROS_INFO("Initialized");
    ros::Rate r(30);
    ros::spin();

    return 0;
}
