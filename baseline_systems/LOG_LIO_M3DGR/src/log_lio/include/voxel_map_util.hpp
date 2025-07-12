#ifndef VOXEL_MAP_UTIL_HPP
#define VOXEL_MAP_UTIL_HPP
//#include "common_lib.h"
#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
//#include <execution>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <mutex>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include "ikd-Tree/ikd_Tree.h"

using std::hash;
using std::vector;
using std::unordered_map;
using std::cout;
using std::printf;
using std::mutex;
#define HASH_P 116101
#define MAX_N 10000000000

    static int plane_id = 0;

// a point to plane matching structure
    typedef struct ptpl {
        Eigen::Vector3d point;
        Eigen::Vector3d point_world;
        Eigen::Vector3d normal;
        Eigen::Vector3d center;
        Eigen::Matrix<double, 6, 6> plane_cov;
        double d;
        int layer;
        Eigen::Matrix3d cov_lidar;
    } ptpl;

// 3D point with covariance
    typedef struct pointWithCov {
        Eigen::Vector3d point;
        Eigen::Vector3d point_world;
        Eigen::Matrix3d cov;
        Eigen::Matrix3d cov_lidar;
    } pointWithCov;

    typedef struct Plane {
        Eigen::Vector3f center;
        Eigen::Vector3f normal;
        Eigen::Vector3f y_normal; //evalsMid
        Eigen::Vector3f x_normal; //evalsMax
        Eigen::Matrix3f covariance;
        Eigen::Matrix<float, 6, 6> plane_cov; //cov (n, q), q = mean_p
        float radius = 0; // sqrt(evalsReal(evalsMax))
        float min_eigen_value = 1;
        float mid_eigen_value = 1;
        float max_eigen_value = 1;
        float d = 0; // plane: n * p + d = 0
        int points_size = 0;

        bool is_plane = false;
        bool is_init = false;
        int id;
        // is_update and last_update_points_size are only for publish plane
        bool is_update = false;
        int last_update_points_size = 0;
        bool update_enable = true;
    } Plane;

class VoxelInfo;

    class VOXEL_LOC {
    public:
        int64_t x, y, z;

        VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
                : x(vx), y(vy), z(vz) {}

        VoxelInfo* info_ptr;
        bool operator==(const VOXEL_LOC &other) const {
            return (x == other.x && y == other.y && z == other.z);
        }
    };


// Hash value
    namespace std {
        template<>
        struct hash<VOXEL_LOC> {
            int64_t operator()(const VOXEL_LOC &s) const {
                using std::hash;
                using std::size_t;
                return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
            }
        };
    } // namespace std

    class OctoTree {
    public:
        vector<pointWithCov> temp_points_; // all points in an octo tree
        vector<pointWithCov> new_points_;  // new points in an octo tree
        Plane *plane_ptr_;
        int max_layer_;
        bool indoor_mode_;
        int layer_;
        int octo_state_; // 0 is end of tree, 1 is not
        OctoTree *leaves_[8];
        double voxel_center_[3]; // x, y, z
        vector<int> layer_point_size_;
        float quater_length_;
        float planer_threshold_;
        int max_plane_update_threshold_;
        int update_size_threshold_;
        int all_points_num_;
        int new_points_num_;
        int max_points_size_;
        int max_cov_points_size_;
        bool init_octo_;
        bool update_cov_enable_;
        bool update_enable_;

        OctoTree(int max_layer, int layer, vector<int> layer_point_size,
                 int max_point_size, int max_cov_points_size, float planer_threshold)
                : max_layer_(max_layer), layer_(layer),
                  layer_point_size_(layer_point_size), max_points_size_(max_point_size),
                  max_cov_points_size_(max_cov_points_size),
                  planer_threshold_(planer_threshold) {
            temp_points_.clear();
            octo_state_ = 0;
            new_points_num_ = 0;
            all_points_num_ = 0;
            // when new points num > 5, do a update
            update_size_threshold_ = 5;
            init_octo_ = false;
            update_enable_ = true;
            update_cov_enable_ = true;
            max_plane_update_threshold_ = layer_point_size_[layer_];
            for (int i = 0; i < 8; i++) {
                leaves_[i] = nullptr;
            }
            plane_ptr_ = new Plane;
        }
    };
#endif