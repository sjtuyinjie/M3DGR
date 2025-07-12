#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include "ring_fals//range_image.h"
#include <point_type.h>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
//typedef ikdTree_PointType PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELODYNE, OUSTER}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);
  void initNormalEstimator();

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI::Ptr cloud_with_normal;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;

    // parameters for normal estimation
    RangeImage range_image;
    bool compute_table = false;
    int Horizon_SCAN, image_cols;
    string ring_table_dir;

    bool runtime_log = false;
    bool compute_normal = false;

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  void projectPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
  void computeRingNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
  void extractCloudAndNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
  void flipNormalsTowardCenterAndNormalized(const cv::Vec3f& center, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, cv::Mat& image_normals);
  void NormalizeNormals(cv::Mat& image_normals);
  void saveNormalPCD(const std::string& file_name, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, const cv::Mat& normals_mat);
  void estimateNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
  void estimateNormals(const pcl::PointCloud<ousterPointXYZIRT>::Ptr& cloud);
  bool pointInImage(const PointXYZIRT& point, int& row, int& col);

  void ouster2velodyne(const pcl::PointCloud<ouster_ros::Point>& cloud_in, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud_out);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;

    // parameters for normal estimation
    cv::Mat rangeMat;
    cv::Mat normals; //record ring normals
    cv::Mat plane_residual; //record ring normals residual in normal estimation
    std::vector<int> image2cloud;
    std::vector<int> cloud2image;
    double aver_normal_time = 0.0, aver_proj_time = 0.0, aver_compu_time = 0.0, aver_smooth_time = 0.0;
    double proj_time = 0.0, compu_time = 0.0, smooth_time = 0.0;
    int num_scans = 0;
    float ang_res_x;

};
