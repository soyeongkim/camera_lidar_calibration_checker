#include "cam_lidar_visualization.hpp"

LiDAR2CamProjection::LiDAR2CamProjection():
pcptr_input_point_cloud_(new pcl::PointCloud<pcl::PointXYZI>), checker_b_image_data_exist_(false), checker_b_velodyne_exist_(false)
{
    nh.getParam("/cam_lidar_visualization/param_d_extrinsic_calibration_roll_deg_"         , param_d_extrinsic_calibration_roll_deg_);
    nh.getParam("/cam_lidar_visualization/param_d_extrinsic_calibration_pitch_deg_"         , param_d_extrinsic_calibration_pitch_deg_);
    nh.getParam("/cam_lidar_visualization/param_d_extrinsic_calibration_yaw_deg_"         , param_d_extrinsic_calibration_yaw_deg_);
    nh.getParam("/cam_lidar_visualization/param_d_extrinsic_calibration_x_m_"         , param_d_extrinsic_calibration_x_m_);
    nh.getParam("/cam_lidar_visualization/param_d_extrinsic_calibration_y_m_"         , param_d_extrinsic_calibration_y_m_);
    nh.getParam("/cam_lidar_visualization/param_d_extrinsic_calibration_z_m_"         , param_d_extrinsic_calibration_z_m_);

    rossub_image_ = nh.subscribe("/cam2/pylon_camera_node/image_raw", 1, &LiDAR2CamProjection::CallbackImage, this);
    rossub_velodyne_ = nh.subscribe("/velodyne_points", 1, &LiDAR2CamProjection::CallBackVelodyne, this);
}

LiDAR2CamProjection::~LiDAR2CamProjection()
{
}

void LiDAR2CamProjection::Run()
{
    std::vector<cv::Point2d> vec_ref_position_point_projected_image;
    
    if (checker_b_image_data_exist_ && checker_b_velodyne_exist_)
    {
        PointCloud2DProjection(pcptr_input_point_cloud_, vec_ref_position_point_projected_image); // for visualization
        ReferenceImagePointViewer(vec_ref_position_point_projected_image);
    }   
}

void LiDAR2CamProjection::CallbackImage(const sensor_msgs::ImageConstPtr& input)
{
    // Point2Image->SetInputImage(input);
    checker_b_image_data_exist_ = true;
    imgmsg_input_image_ = input;
    cvptr_input_image_ = cv_bridge::toCvCopy(imgmsg_input_image_, sensor_msgs::image_encodings::BGR8);

    cv::Mat cvp_image = cvptr_input_image_->image;
    d_input_image_size_ = cvp_image.cols * cvp_image.rows;
}

void LiDAR2CamProjection::CallBackVelodyne(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    checker_b_velodyne_exist_ = true;
    sensor_msgs::PointCloud2 pc2_output_point_cloud;
    ROS_INFO_STREAM("CALLBACK VELODYNE");

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcptr_point_cloud_input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcptr_point_cloud_input);
    pcptr_input_point_cloud_ = pcptr_point_cloud_input;
}

void LiDAR2CamProjection::ReferenceImagePointViewer(std::vector<cv::Point2d> vec_input_points)
{
    cv::Mat cvmat_image_origin = cvptr_input_image_->image;
    cv::Mat cvmat_image_projection = cvmat_image_origin.clone();

    ROS_INFO_STREAM("vec_input_points.size() :"<<vec_input_points.size());
    
    for (size_t i = 0; i < vec_input_points.size(); ++i)
    {
        cv::Point2d cvp_point_2d = vec_input_points[i];
        cv::rectangle(cvmat_image_projection,cv::Point(cvp_point_2d.x ,cvp_point_2d.y ),
                                        cv::Point(cvp_point_2d.x ,cvp_point_2d.y ),
                                        CV_RGB(255,0,0),
                                        3);
    }
    cv::imshow("Ref Whole Point Cloud Image", cvmat_image_projection);
    cv::waitKey(1);
}

void LiDAR2CamProjection::PointCloud2DProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr inputPointCloud, std::vector<cv::Point2d>& vec_output_2d_projected_point)
{
    if(cvptr_input_image_ == NULL)
    {
        ROS_ERROR_STREAM("!!! IMAGE DATA DOES NOT EXIST !!!");
        return;
    }

    if(inputPointCloud->points.size()<1)
    {
        ROS_INFO_STREAM("NO INPUT CLOUDS");
        return;
    }

    // LiDAR-Camera Extrinsic Caliibration

    Eigen::Matrix4d egmat_cam_to_lidar_transform_mat;
    Eigen::Matrix4d egmat_cam_to_particle_transform_mat;

    double calibration_d_roll_deg  = param_d_extrinsic_calibration_roll_deg_* M_PI/180.;
    double calibration_d_pitch_deg = param_d_extrinsic_calibration_pitch_deg_ * M_PI/180.;
    double calibration_d_yaw_deg   = param_d_extrinsic_calibration_yaw_deg_ * M_PI/180.;

    double calibration_tx_m = param_d_extrinsic_calibration_x_m_;
    double calibratoin_ty_m = param_d_extrinsic_calibration_y_m_;
    double calibration_tz_m = param_d_extrinsic_calibration_z_m_;

    egmat_cam_to_lidar_transform_mat << cos(calibration_d_pitch_deg)*cos(calibration_d_yaw_deg), -cos(calibration_d_roll_deg)*sin(calibration_d_yaw_deg) + sin(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*cos(calibration_d_yaw_deg),  sin(calibration_d_roll_deg)*sin(calibration_d_yaw_deg)+cos(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*cos(calibration_d_yaw_deg), calibration_tx_m,
                                        cos(calibration_d_pitch_deg)*sin(calibration_d_yaw_deg),  cos(calibration_d_roll_deg)*cos(calibration_d_yaw_deg) + sin(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*sin(calibration_d_yaw_deg), -sin(calibration_d_roll_deg)*cos(calibration_d_yaw_deg)+cos(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*sin(calibration_d_yaw_deg), calibratoin_ty_m, 
                                        -sin(calibration_d_pitch_deg)         ,  sin(calibration_d_roll_deg)*cos(calibration_d_pitch_deg)                              ,  cos(calibration_d_roll_deg)*cos(calibration_d_pitch_deg)                            , calibration_tz_m, 
                                        0.                 , 0.                                                 , 0.                                               , 1.;


    cv::Mat cv_rotation_mat, cv_translate_vec;

    cv::Mat cv_camera_intrinsic_mat(cv::Size(3,3), CV_64F, calibration_array_camera_intrinsic_matrix_);
    cv::Mat cv_camera_distortion_mat(cv::Size(1,5), CV_64F, calibration_array_camera_distortion_matrix_);
    cv::Mat cv_camera_extrinsic_mat;

    cv::eigen2cv(egmat_cam_to_lidar_transform_mat,cv_camera_extrinsic_mat);

    cv::Mat R_zero          (cv::Size(3,3), CV_64F, calibration_array_zero_rotation_matrix_);
    cv::Mat t_zero          (cv::Size(3,1), CV_64F, calibration_array_zero_translation_vector_);
    cv::Mat cvmat_image_origin    = cvptr_input_image_->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcptr_rotated_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    //read cv_rotation_mat t from Extrin_matrix
    cv_rotation_mat    = cv_camera_extrinsic_mat(cv::Range(0,3),cv::Range(0,3));
    cv_translate_vec   = cv_camera_extrinsic_mat(cv::Range(0,3),cv::Range(3,4));

    cv_rotation_mat    = cv_rotation_mat.t();
    cv_translate_vec   = -cv_rotation_mat * cv_translate_vec;

    //rotate the cloud
    Eigen::Matrix3d egmat_to_camera_rotation;
    cv::cv2eigen(cv_rotation_mat, egmat_to_camera_rotation);

    Eigen::Translation3d egmat_to_camera_translation(cv_translate_vec.at<double>(0,0),
                                            cv_translate_vec.at<double>(1,0),
                                            cv_translate_vec.at<double>(2,0));
    Eigen::Matrix4d egmat_to_camera_transformation = (egmat_to_camera_translation * egmat_to_camera_rotation).matrix();

    //filter
    std::vector<cv::Point3d> vec_3D_points;
    pcl::transformPointCloud (*inputPointCloud, *pcptr_rotated_point_cloud, egmat_to_camera_transformation);

    for (size_t i = 0; i < inputPointCloud->size(); ++i)
    {
        pcl::PointXYZI pt_3d_point = inputPointCloud->points[i]; // for filter
        pcl::PointXYZI pt_3d_point_transformed = pcptr_rotated_point_cloud->points[i]; // for rotation(depth color)
        if (pt_3d_point.x > -i_pointcloud_2d_projection_range_m_ && pt_3d_point.x < i_pointcloud_2d_projection_range_m_ && pt_3d_point.y > -i_pointcloud_2d_projection_range_m_ && pt_3d_point.y < i_pointcloud_2d_projection_range_m_ && pt_3d_point_transformed.z > 0)
        {
            vec_3D_points.emplace_back(cv::Point3d(pt_3d_point.x, pt_3d_point.y, pt_3d_point.z));
            double d_current_distance = pt_3d_point_transformed.z;

            d_max_range_projected_point_cloud_m_ = (d_current_distance > d_max_range_projected_point_cloud_m_)? d_current_distance: d_max_range_projected_point_cloud_m_;
            d_min_range_projected_point_cloud_m_ = (d_current_distance < d_min_range_projected_point_cloud_m_)? d_current_distance: d_min_range_projected_point_cloud_m_;
        }
    }

    // project 3d-points into image view
    if(vec_3D_points.size() > 0)
    {
        std::vector<cv::Point2d> vec_2d_projected_point;
        cv::projectPoints(vec_3D_points, cv_rotation_mat, cv_translate_vec, cv_camera_intrinsic_mat, cv_camera_distortion_mat, vec_2d_projected_point);

        vec_output_2d_projected_point = vec_2d_projected_point;
    }
}

int main(int argc, char **argv)
{
    std::string str_nodename = "cam_lidar_visualization";
	ros::init(argc, argv, str_nodename);
	ros::NodeHandle nh;

    LiDAR2CamProjection LiDAR2CamProjection;

    int i_init_loop_frequency = 10;

    ros::Rate rosrate_loop_rate(i_init_loop_frequency);

    while(ros::ok())
    {
            ros::spinOnce();
            LiDAR2CamProjection.Run();
            rosrate_loop_rate.sleep();
    }

    return 0;
}