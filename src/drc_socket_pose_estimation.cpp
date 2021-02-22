// This code is using Kinect
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include <image_geometry/pinhole_camera_model.h>
#include <vector>

#include <iostream>
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
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/impl/search.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;


class SocketPoseEstimation
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber pc_sub_;
  ros::Publisher point_cloud_pub_, seg_point_cloud_pub_;
  ros::Publisher marker_pub;
 
  image_geometry::PinholeCameraModel pin_hole_camera;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_, cloud_transformed_, cloud_filtered_;
  pcl::ExtractIndices<pcl::PointXYZ> extract_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  std::string fixed_frame_;
  tf::TransformListener listener_;
  tf::StampedTransform transform;
  std::vector<int> circle_center_x;
  std::vector<int> circle_center_y;
  std::vector<float> center_x;
  std::vector<float> center_y;
  std::vector<float> center_z;
  bool isProjectPixelTo3dRay = false;
  float roll, yaw, pitch;
  tf::TransformBroadcaster br1, br2;

public:
  SocketPoseEstimation()
    : it_(nh_), cloud_transformed_(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>), cloud_hull(new pcl::PointCloud<pcl::PointXYZ>), cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    // Subscrive to input video feed and publish output video feed
    info_sub_ = nh_.subscribe("/kinect2/qhd/camera_info", 10, &SocketPoseEstimation::info_callback, this);
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, &SocketPoseEstimation::imageCb, this);
    depth_sub_ = it_.subscribe("/kinect2/qhd/image_depth_rect", 1, &SocketPoseEstimation::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    pc_sub_ = nh_.subscribe("/kinect2/qhd/points", 1, &SocketPoseEstimation::pointcloudcb, this);
    point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed_point_cloud", 1000);
    seg_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("segmented_plane_point_cloud", 1000);
    fixed_frame_ = "/root";

  }

  ~SocketPoseEstimation()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    pin_hole_camera.fromCameraInfo(msg);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

// hough circle detection
    Mat circle_gray;
    cv::cvtColor(cv_ptr->image, circle_gray, CV_BGR2GRAY); // convert image to gray
    GaussianBlur(circle_gray, circle_gray, Size(9, 9), 2, 2 ); // Reduce the noise so we avoid false circle detection
    vector<Vec3f> circles;
    HoughCircles(circle_gray, circles, CV_HOUGH_GRADIENT, 1, circle_gray.rows/100, 200, 40, 0, 100 );// Apply the Hough Transform to find the circles
    // push back non-duplicated circle only
    bool isDuplicated = false;
    for(size_t i = 0; i < circles.size(); i++)  // Draw the circles
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle(cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0);
      std::cout << "center-" << i << "(x,y): " << circles[i][0] << ", " << circles[i][1] << std::endl;
      if (circle_center_x.size() == 0)
      {
        circle_center_x.push_back(circles[i][0]);
        circle_center_y.push_back(circles[i][1]);
      }
      else
      {
        for (int j = 0; j < circle_center_x.size(); j++)
        {
          if (abs(circle_center_x[j] - circles[i][0]) < 10)
          {
            isDuplicated = true;
            break;
          }
        }
        if (isDuplicated == false)
        {
          circle_center_x.push_back(circles[i][0]);
          circle_center_y.push_back(circles[i][1]);
        }
      }

      std::cout << "size of the center vector: " << circle_center_x.size()<< std::endl;
      // circle outline
      circle(cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0);
    } 
    for (int i = 0; i < circle_center_x.size(); i++)
    {
      std::cout << "center of circles(x,y): " << circle_center_x[i] << "," << circle_center_y[i] << std::endl;
    }
    cv::imshow("Hough Circle", cv_ptr->image);
    cv::waitKey(300);
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImageConstPtr depth_img_cv;
    try
    {
    depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (circle_center_x.size() == 2 && isProjectPixelTo3dRay == false) // only calculate once
    {
      std::cout << "ready to calculate the 3d positions of the circles" << std::endl;
      isProjectPixelTo3dRay = true;
        for (int i = 0; i < circle_center_x.size(); i++){
        cv::Point2d pixel_point(circle_center_x[i], circle_center_y[i]); 
        float depth = depth_img_cv->image.at<short int>(pixel_point);
        cout << "depth" << depth<< endl;
        if (depth > 0.2){
          cv::Point3d xyz =pin_hole_camera.projectPixelTo3dRay(pixel_point);
          // cout << xyz<<endl;
          cv::Point3d coordinate = xyz * depth;
          float co_x_temp = coordinate.x/1000;
          float co_y_temp = coordinate.y/1000;
          float co_z_temp = coordinate.z/1000;
          center_x.push_back(co_x_temp);
          center_y.push_back(co_y_temp);
          center_z.push_back(co_z_temp);
          
         } 
       }
    }

    for (int i = 0; i < center_x.size(); i++)
    {
      std::cout << "3D Positions of the circle centers: " << center_x[i] << "," << center_y[i] << "," << center_z[i] << std::endl;
    } 

  } 

  void pointcloudcb(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
  {
      geometry_msgs::Point plane_normal;
      pcl::ExtractIndices<pcl::PointXYZ> extract_;
      pcl::ConvexHull<pcl::PointXYZ> chull;
      cloud_raw_ = msg;
      // cloud_transformed_ = cloud_raw_;
      // listener_.waitForTransform(fixed_frame_, "/kinect2_rgb_optical_frame", (ros::Time)(*cloud_raw_).header.stamp, ros::Duration(3.0));
      bool transform_success = pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed_, listener_);
      //std::cout << transform_success << std::endl;
      point_cloud_pub_.publish(cloud_transformed_);

      if (transform_success == 0) 
      {
        ROS_INFO("failed transform point cloud");
        return;
      }

      
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

      // Get plane coefficients
      // coefficients->values[0]

      // Get plane normal
      float length = sqrt(coefficients->values[0] * coefficients->values[0] +
                          coefficients->values[1] * coefficients->values[1] +
                          coefficients->values[2] * coefficients->values[2]);
      plane_normal.x = coefficients->values[0] / length;
      plane_normal.y = coefficients->values[1] / length;
      plane_normal.z = coefficients->values[2] / length;
      std::cout << "normal: " << plane_normal.x << "," << plane_normal.y << "," << plane_normal.z << std::endl;

      if (center_x.size() == 2)
      {
        // roll = 0;
        // pitch = atan((sqrt(pow((center[0] - plane_normal.x),2)+pow((center[1] - plane_normal.y),2)))/(center[2] - plane_normal.z));
        // yaw = atan((center[0] - plane_normal.x)/(plane_normal.y - center[1]));  
        
        // if roll = 0 
        // roll = 0;
        // pitch = atan((sqrt(pow(plane_normal.x,2)+pow(plane_normal.y,2)))/abs(plane_normal.z));
        // yaw = atan(-plane_normal.x/plane_normal.y); 

        // if pitch = 0, ignore the rotation about the Y axis
        roll = atan(plane_normal.z/plane_normal.y);
        pitch = 0; 
        yaw = atan(-plane_normal.x/sqrt(pow(plane_normal.y,2)+pow(plane_normal.z,2)));
        tf::Transform transform1, transform2;
        transform1 = tf_transform_calculation(center_x[0], center_y[0], center_z[0]); 
        transform2 = tf_transform_calculation(center_x[1], center_y[1], center_z[1]);
        transform1.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw)); 
        transform2.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));  
        // publish tf
        br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "root", "socket1"));
        br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "root", "socket2"));
      }

  }  

  tf::Transform tf_transform_calculation(float x, float y, float z)
  {
    //calculate transform from /endeffector to /object
    tf::TransformListener listener;
    tf::StampedTransform stampedtransform_transform;
    listener.waitForTransform("/root", "/kinect2_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/root", "/kinect2_rgb_optical_frame", ros::Time(0), stampedtransform_transform); // target frame to source frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::Transform ntransform;
    ntransform = stampedtransform_transform * transform;
    return ntransform;
  }

 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "socket_pose_estimation");
  SocketPoseEstimation SPE;
  ros::spin();
  return 0;
}
