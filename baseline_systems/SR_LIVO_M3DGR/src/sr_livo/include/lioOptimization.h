#pragma once
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_set>

// ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

// eigen 
#include <Eigen/Core>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>	
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

// livox
#include <livox_ros_driver/CustomMsg.h>

#include "cloudMap.h"

// cloud processing
#include "cloudProcessing.h"

// image processing
#include "imageProcessing.h"

// IMU processing
#include "state.h"

// eskf estimator
#include "eskfEstimator.h"

// utility
#include "utility.h"
#include "parameters.h"

class imageProcessing;

struct Measurements
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<sensor_msgs::ImuConstPtr> imu_measurements;

    std::pair<double, double> time_sweep;
    std::vector<point3D> lidar_points;

    double time_image = 0.0;
    cv::Mat image;

    bool rendering = false;
};

class cloudFrame
{
public:
    double time_sweep_begin, time_sweep_end;
    double time_frame_begin, time_frame_end;

    int id;   // the index in all_cloud_frame
    int sub_id;    // the index of segment
    int frame_id;

    state *p_state;

    std::vector<point3D> point_frame;

    double offset_begin;
    double offset_end;
    double dt_offset;

    bool success;

    cloudFrame(std::vector<point3D> &point_frame_, state *p_state_);

    cloudFrame(cloudFrame *p_cloud_frame);

    void release();

    cv::Mat rgb_image;
    cv::Mat gray_image;

    int image_cols;
    int image_rows;

    bool if2dPointsAvailable(const double &u, const double &v, const double &scale = 1.0, double fov_mar = -1.0);

    bool getRgb(const double &u, const double &v, int &r, int &g, int &b);

    Eigen::Vector3d getRgb(double &u, double &v, int layer, Eigen::Vector3d *rgb_dx = nullptr, Eigen::Vector3d *rgb_dy = nullptr);

    bool project3dTo2d(const pcl::PointXYZI &point_in, double &u, double &v, const double &scale);

    bool project3dPointInThisImage(const pcl::PointXYZI &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale);

    bool project3dPointInThisImage(const Eigen::Vector3d &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale);

    void refreshPoseForProjection();
};

struct Neighborhood {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d center = Eigen::Vector3d::Zero();

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

    double a2D = 1.0; // Planarity coefficient
};

struct ResidualBlock {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d point_closest;

    Eigen::Vector3d pt_imu;

    Eigen::Vector3d normal;

    double alpha_time;

    double weight;
};

class estimationSummary {

public:

    int sample_size = 0; // The number of points sampled

    int number_of_residuals = 0; // The number of keypoints used for ICP registration

    int robust_level = 0;

    double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

    double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

    double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end

    double ego_orientation = 0.0; // The angular distance between the beginning and the end of the frame

    bool success = true; // Whether the registration was a success

    std::string error_message;

    estimationSummary();

    void release();

};

struct optimizeSummary {

    bool success = false; // Whether the registration succeeded

    int num_residuals_used = 0;

    std::string error_log;
};

enum IMAGE_TYPE{RGB8 = 1, COMPRESSED = 2};

class lioOptimization{

private:

	ros::NodeHandle nh;

	ros::Publisher pub_cloud_body;     // the registered cloud of cuurent sweep to be published for visualization
    ros::Publisher pub_cloud_world;   // the cloud of global map to be published for visualization
    ros::Publisher pub_odom;		// the pose of current sweep after LIO-optimization
    ros::Publisher pub_path;				// the position of current sweep after LIO-optimization for visualization
    ros::Publisher pub_cloud_color;
    std::vector<std::shared_ptr<ros::Publisher>> pub_cloud_color_vec;

    ros::Subscriber sub_cloud_ori;   // the data of original point clouds from LiDAR sensor
    ros::Subscriber sub_imu_ori;			// the data of original accelerometer and gyroscope from IMU sensor
    ros::Subscriber sub_img_ori;
    
    int image_type;

    std::string lidar_topic;
    std::string imu_topic;
    std::string image_topic;

	cloudProcessing *cloud_pro;
    eskfEstimator *eskf_pro;
    imageProcessing *img_pro;

    bool extrin_enable;

    double laser_point_cov;

    std::vector<double> v_G;
    std::vector<double> v_extrin_t_imu_lidar;
    std::vector<double> v_extrin_R_imu_lidar;

    Eigen::Matrix3d R_imu_lidar;
    Eigen::Vector3d t_imu_lidar;

    std::vector<double> v_extrin_t_imu_camera;
    std::vector<double> v_extrin_R_imu_camera;

    Eigen::Matrix3d R_imu_camera;
    Eigen::Vector3d t_imu_camera;

    std::vector<double> v_camera_intrinsic;
    std::vector<double> v_camera_dist_coeffs;

    Eigen::Vector3d pose_lid;

    std::queue<std::pair<double, double>> time_lidar_buffer;
    std::queue<double> time_img_buffer;

    std::queue<std::vector<point3D>> lidar_buffer;
    std::queue<sensor_msgs::Imu::ConstPtr> imu_buffer;
    std::queue<cv::Mat> img_buffer;
    std::queue<point3D> point_buffer;

    std::vector<cloudFrame*> all_cloud_frame;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr down_cloud_body;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr down_cloud_world;

    std::vector<std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>>>  nearest_points;

	double last_time_lidar;
	double last_time_imu;
    double last_time_img;

    double last_get_measurement;
    bool last_rendering;
    double last_time_frame;
    double current_time;

    int index_frame;

    double time_max_solver;
    int num_max_iteration;

    // lidar range
    float det_range;
    double fov_deg;

    voxelHashMap voxel_map;
    voxelHashMap color_voxel_map;
    Hash_map_3d<long, rgbPoint*> hashmap_3d_points;

    odometryOptions odometry_options;
    mapOptions map_options;

    geometry_msgs::Quaternion geoQuat;
    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Path path;
    nav_msgs::Odometry odomAftMapped;

    double dt_sum;

    std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_meas;
    std::vector<imuState> imu_states;

    std::vector<voxelId> voxels_recent_visited_temp;

public:

    // initialize class
	lioOptimization();

    void readParameters();

    void allocateMemory();

    void initialValue();
    // initialize class

    // get sensor data
    void livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);

    void standardCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

	void imuHandler(const sensor_msgs::Imu::ConstPtr &msg);

    void imageHandler(const sensor_msgs::ImageConstPtr &msg);

    void compressedImageHandler(const sensor_msgs::CompressedImageConstPtr &msg);
    // get sensor data

    // main loop
    std::vector<Measurements> getMeasurements();

    void process(std::vector<point3D> &cut_sweep, double timestamp_begin, double timestamp_offset, cv::Mat &cur_image, bool to_rendering);

	void run();
    // main loop

    // data handle and state estimation
    cloudFrame* buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset);

    void makePointTimestamp(std::vector<point3D> &sweep, double time_begin, double time_end);

    void stateInitialization(state *cur_state);

    optimizeSummary stateEstimation(cloudFrame *p_frame, bool to_rendering);

    optimizeSummary optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, double sample_voxel_size);

    optimizeSummary buildPlaneResiduals(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, std::vector<planeParam> &plane_residuals, cloudFrame *p_frame, double &loss_sum);

    optimizeSummary updateIEKF(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame);

    Neighborhood computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> searchNeighbors(voxelHashMap &map, const Eigen::Vector3d &point,
        int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);
    // data handle and state estimation

    // map update
    void addPointToMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, 
    int min_num_points, cloudFrame* p_frame);

    void addPointToColorMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, 
    int min_num_points, cloudFrame* p_frame, std::vector<voxelId> &voxels_recent_visited_temp);

    void addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0, bool to_rendering = false);

    void removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance);

    size_t mapSize(const voxelHashMap &map);
    // map update

    // save result to device
    void recordSinglePose(cloudFrame *p_frame);
    // save result to device

    // publish result by ROS for visualization
    void publish_path(ros::Publisher pub_path,cloudFrame *p_frame);

    void set_posestamp(geometry_msgs::PoseStamped &body_pose_out,cloudFrame *p_frame);

    void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, rgbPoint &point, cloudFrame *p_frame);

    void publishCLoudWorld(ros::Publisher & pub_cloud_world, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes, cloudFrame* p_frame);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;
    void publish_odometry(const ros::Publisher & pubOdomAftMapped, cloudFrame *p_frame);

    void pubColorPoints(ros::Publisher &pub_cloud_rgb, cloudFrame *p_frame);

    void threadPubColorPoints();
    // publish result by ROS for visualization

    void saveColorPoints();

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;
};
