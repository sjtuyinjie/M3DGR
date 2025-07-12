#ifndef LIDAR_ODOM_H__
#define LIDAR_ODOM_H__
#include "common/timer/timer.h"

#include "algo/eskf.hpp"
#include "algo/iekf.hpp"
#include "algo/static_imu_init.h"
#include "liw/lio_utils.h"

#include <condition_variable>

#include "tools/tool_color_printf.hpp"
#include "common/timer/timer.h"
#include "common/utility.h"
#include "tools/point_types.h"
#include "algo/IMU_Processing.hpp"

#include <sys/times.h>
#include <sys/vtimes.h>
#include <omp.h>

namespace zjloc
{

     struct liwOptions
     {
          double surf_res;
          bool log_print;
          bool extrinsic_est_en;
          int num_max_iterations;
          //   ct_icp
          double size_voxel_map;
          double min_distance_points;
          int max_num_points_in_voxel;
          double max_distance;
          double weight_alpha;
          double weight_neighborhood;
          double max_dist_to_plane_icp;
          int init_num_frames;
          int voxel_neighborhood;
          int max_number_neighbors;
          int threshold_voxel_occupancy;
          bool estimate_normal_from_neighborhood;
          int min_number_neighbors;
          double power_planarity;
          int num_closest_neighbors;

          double sampling_rate;

          int max_num_residuals;
          int min_num_residuals;

          double thres_orientation_norm;
          double thres_translation_norm;
     };

     class lidarodom
     {
     public:
          lidarodom(/* args */);
          ~lidarodom();

          bool init(const std::string &config_yaml);

          void pushData(std::vector<point3D>, std::pair<double, double> data);
          void pushData(IMUPtr imu);

          void run();

          int getIndex() { return index_frame; }

          void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) { pub_cloud_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun) { pub_pose_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) { pub_data_to_ros = fun; }

     private:
          void loadOptions();

          std::vector<MeasureGroup> getMeasureMents();

          /// 处理同步之后的IMU和雷达数据
          void ProcessMeasurements(MeasureGroup &meas);

          void poseEstimation();

          void optimize();

          void lasermap_fov_segment();

          void map_incremental(int min_num_points = 0);

          void addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                             const double &intensity, double voxel_size,
                             int max_num_points_in_voxel, double min_distance_points,
                             int min_num_points);

          void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points,
                             const Eigen::Vector3d &point, const double &intensity);

          // search neighbors
          Neighborhood computeNeighborhoodDistribution(
              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors,
                          int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

          inline Sophus::SO3d r2SO3(const Eigen::Vector3d r)
          {
               return Sophus::SO3d::exp(r);
          }

          void visualize_state();

     private:
          /// @brief 数据
          std::string config_yaml_;
          SE3 TIL_;                   //   lidar 转换到 imu
          MeasureGroup measures_;     // sync IMU and lidar scan
          bool imu_need_init_ = true; // 是否需要估计IMU初始零偏
          int index_frame = 1;
          liwOptions options_;

          std::shared_ptr<ImuProcess> p_imu;

          double opt_cnt = 0;
          double curr_time = 0;
          double total_computation = 0;

          voxelHashMap voxel_map;
          Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); //   lidar 转换到 imu坐标系下
          Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();     //   need init
          double laser_point_cov = 0.001;

          double deltaT, deltaR, first_lidar_time = 0;
          double total_distance = 0.0;
          Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
          bool flg_EKF_inited = false;
          Eigen::Vector3d position_last = Eigen::Vector3d(0, 0, 0);

          /// @brief 滤波器
          IESKFD eskf_;
          std::vector<NavStated> imu_states_; // ESKF预测期间的状态
          IMUPtr last_imu_ = nullptr;
          StatesGroup state;

          double time_curr;
          double delay_time_;
          Vec3d g_{0, 0, -9.8};

          /// @brief 数据管理及同步
          std::deque<std::vector<point3D>> lidar_buffer_;
          std::deque<IMUPtr> imu_buffer_;    // imu数据缓冲
          double last_timestamp_imu_ = -1.0; // 最近imu时间
          double last_timestamp_lidar_ = 0;  // 最近lidar时间
          std::deque<std::pair<double, double>> time_buffer_;

          /// @brief mutex
          std::mutex mtx_buf;
          std::mutex mtx_state;
          std::condition_variable cond;

          std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> pub_cloud_to_ros;
          std::function<bool(std::string &topic_name, SE3 &pose, double time)> pub_pose_to_ros;
          std::function<bool(std::string &topic_name, double time1, double time2)> pub_data_to_ros;
          pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;
     };
}

#endif