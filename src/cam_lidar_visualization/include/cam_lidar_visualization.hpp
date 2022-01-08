#include <string>
#include <deque>
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <map>
#include <random>
#include <Eigen/Dense>
#include <chrono>

// ROS header
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ros/transport_hints.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/kdtree/kdtree_flann.h>

// Utility header
#include <pcl_conversions/pcl_conversions.h> // pcl_conversions::toPCL
#include <pcl/point_types.h>                 // pcl::PointXYZ
#include <pcl/PCLPointCloud2.h>              // pcl::PCLPointCloud2
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl_ros/transforms.h>

#include"eigen3/Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/eigen.hpp"

// Message header
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>

#include <ublox_msgs/HnrPVT.h>
#include <visualization_msgs/Marker.h>

class LiDAR2CamProjection
{
    public:
        LiDAR2CamProjection();
        ~LiDAR2CamProjection();

        void Run();
    
    private:
        double param_d_extrinsic_calibration_roll_deg_;
        double param_d_extrinsic_calibration_pitch_deg_;
        double param_d_extrinsic_calibration_yaw_deg_;
        double param_d_extrinsic_calibration_x_m_;
        double param_d_extrinsic_calibration_y_m_;
        double param_d_extrinsic_calibration_z_m_;

        ros::Subscriber rossub_image_;
        ros::Subscriber rossub_velodyne_;

        ros::NodeHandle nh;

        cv_bridge::CvImagePtr cvptr_input_image_;
        sensor_msgs::ImageConstPtr imgmsg_input_image_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcptr_input_point_cloud_;

        bool checker_b_image_data_exist_;
        bool checker_b_velodyne_exist_;

        double calibration_array_camera_intrinsic_matrix_[9]         = {400.00000000000006    , 0.                 , 400., 
                                                                        0.                    , 400.00000000000006 , 300., 
                                                                        0.                    , 0.                 , 1.                };
        double calibration_array_camera_distortion_matrix_[5]      = {0., 0., 0., 0., 0.};
        double calibration_array_zero_rotation_matrix_[9]   = {1., 0., 0.,
                                      0., 1., 0., 
                                      0., 0., 1.};
        double calibration_array_zero_translation_vector_[3]         = {0.,0.,0.};

        int i_pointcloud_2d_projection_range_m_ = 100;
        double d_max_range_projected_point_cloud_m_;
        double d_min_range_projected_point_cloud_m_;
        double d_input_image_size_;

    private:
        void CallbackImage(const sensor_msgs::ImageConstPtr& input);
        void CallBackVelodyne(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void PointCloud2DProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr inputPointCloud, std::vector<cv::Point2d>& vec_output_2d_projected_point);
        void ReferenceImagePointViewer(std::vector<cv::Point2d> vec_input_points);

};
