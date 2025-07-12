// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
// #include <Python.h>
#include <python2.7/Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <common_lib.h>

#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eskf_lio/States.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include "tool_color_printf.hpp"

#define INIT_TIME (0)
#define LASER_POINT_COV (0.0015)
#define NUM_MATCH_POINTS (5)

std::string root_dir = ROOT_DIR;

int iterCount = 0;
int NUM_MAX_ITERATIONS = 0;

int effct_feat_num = 0;

double filter_size_surf_min;

/// IMU relative variables
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool lidar_pushed = false;
bool flg_exit = false;
bool flg_reset = false;
bool flg_first_scan = true;

/// Buffers for measurements
double cube_len = 0.0;
double lidar_end_time = 0.0;
double last_timestamp_lidar = -1;
double last_timestamp_imu = -1;

double res_mean_last = 0.05;
double total_distance = 0.0;
auto position_last = Zero3d;

std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

// surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
PointCloudXYZI::Ptr coeffSel(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterSurf;

bool dense_map_en, flg_EKF_inited = false, flg_EKF_converged = false, extrinsic_est_en = true;
// all points
PointCloudXYZI::Ptr laserCloudFullRes2(new PointCloudXYZI());

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZI>());

KD_TREE<PointType> ikdtree;

Eigen::Vector3f XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
Eigen::Vector3f XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
V3D pos_lid;

// estimator inputs and output;
MeasureGroup Measures;
StatesGroup state;

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

// project lidar frame to world
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
    Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
    Eigen::Vector3d p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity * 10000;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

vector<BoxPointType> cub_needrm;
int kdtree_delete_counter = 0;
double kdtree_delete_time = 0.0;

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);

    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();

    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    lidar_buffer.push_back(msg);
    last_timestamp_lidar = msg->header.stamp.toSec();
    // std::cout << "get lidar: " << last_timestamp_lidar << ",lidar size: " << lidar_buffer.size() << std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // std::cout << "got imu: " << timestamp << " imu size " << imu_buffer.size() << std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push lidar frame ***/
    if (!lidar_pushed)
    {
        meas.lidar.reset(new PointCloudXYZI());
        pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().normal_z; //  normal_z存储timespan
        meas.lidar_end_time = lidar_end_time;
        // std::cout << "timespan: " << meas.lidar->points.back().normal_z << ", point end ratio: " << meas.lidar->points.back().normal_x << std::endl;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    // if (meas.imu.empty()) return false;
    // std::cout<<"[IMU Sycned]: "<<imu_time<<" "<<lidar_end_time<<std::endl;
    return true;
}

vector<PointVector> Nearest_Points;
double filter_size_map_min = 0;
int feats_down_size = 0, add_point_size = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

void saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, Eigen::Vector3d &xyz, Eigen::Quaterniond &xyzw)
{
    fout << stamp << " " << xyz[0] << " " << xyz[1] << " " << xyz[2] << " " << xyzw.x() << " " << xyzw.y() << " " << xyzw.z() << " " << xyzw.w() << std::endl;
}

void saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, double x, double y, double z, double qx, double qy, double qz, double qw)
{
    fout << stamp << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    std::string imu_topic;
    bool save_tum_traj;

    /*** variables initialize ***/
    nh.param<std::string>("common/imuTopic", imu_topic, "/livox/imu");
    nh.param<bool>("mapping/dense_map_enable", dense_map_en, false);
    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 10);
    nh.param<double>("mapping/filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("mapping/filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("mapping/cube_side_length", cube_len, 1000);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());
    nh.param<bool>("mapping/save_tum_traj", save_tum_traj, false);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    std::cout << "R: \n"
              << Lidar_R_wrt_IMU << std::endl;
    std::cout << "T: " << Lidar_T_wrt_IMU.transpose() << std::endl;

    ROS_INFO("\033[1;32m----> ESKF_LIO mapping Started.\033[0m");

    ros::Subscriber sub_pcl = nh.subscribe("/laser_cloud_surf", 20000, feat_points_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 20000, imu_cbk);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 10);

    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/camera_init";

    /*** variables definition ***/

    double deltaT, deltaR, first_lidar_time = 0;
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    nav_msgs::Odometry odomAftMapped;

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    std::shared_ptr<ImuProcess> p_imu(new ImuProcess());
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);

    //  create folder
    std::string command = "mkdir -p " + std::string(ROOT_DIR) + "Log";
    system(command.c_str());
    /*** debug record ***/
    std::ofstream fout_pre, fout_out;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
    if (fout_pre && fout_out)
        std::cout << "~~~~" << ROOT_DIR << " file opened" << std::endl;
    else
        std::cout << "~~~~" << ROOT_DIR << " doesn't exist" << std::endl;

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::spinOnce();
        while (sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
            }
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }

            p_imu->Process(Measures, state, feats_undistort);
            StatesGroup state_propagat(state); //  predict pose
            pos_lid = state_propagat.pos_end + state_propagat.rot_end * state_propagat.T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                std::cout << "No point, not ready for odometry, skip this scan" << std::endl;
                continue;
            }

            std::cout << ANSI_COLOR_YELLOW_BOLD << "-------------------------------------------------------------" << ANSI_COLOR_RESET << std::endl;
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            /*** Compute the euler angle ***/
            V3D euler_cur = RotMtoEuler(state.rot_end);

            std::cout << "pre-integrated states, r:" << euler_cur.transpose() * 57.3 << " ,p:"
                      << state.pos_end.transpose() << " ,v:" << state.vel_end.transpose() << " ,bg:"
                      << state.bias_g.transpose() << " ,ba:" << state.bias_a.transpose() << std::endl;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the features of new frame ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down);

            feats_down_size = feats_down->points.size();
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down->points.size() > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();

            std::cout << ANSI_COLOR_GREEN << "[ mapping ]: Raw feature num: " << feats_undistort->points.size()
                      << " downsamp num: " << feats_down_size << " Map num: " << featsFromMapNum << ANSI_COLOR_RESET << std::endl;

            /*** ICP and iterated Kalman filter update ***/
            PointCloudXYZI::Ptr coeffSel_tmpt(new PointCloudXYZI(*feats_down));
            PointCloudXYZI::Ptr feats_down_updated(new PointCloudXYZI(*feats_down));
            std::vector<double> res_last(feats_down_size, 1000.0); // initial

            if (featsFromMapNum >= 5)
            {

                normvec->resize(feats_down_size);
                feats_down_world->resize(feats_down_size);

                Nearest_Points.resize(feats_down_size);

                std::vector<bool> point_selected_surf(feats_down_size, true);

                int rematch_num = 0;
                bool rematch_en = 0;
                flg_EKF_converged = 0;
                deltaR = 0.0;
                deltaT = 0.0;

                for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++)
                {

                    laserCloudOri->clear();
                    coeffSel->clear();

                    /** closest surface search and residual computation **/
                    // omp_set_num_threads(4);
                    // #pragma omp parallel for
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        PointType &point_body = feats_down->points[i];
                        PointType &point_world = feats_down_updated->points[i];

                        /* transform to world frame */
                        V3D p_body(point_body.x, point_body.y, point_body.z);
                        V3D p_global(state.rot_end * (state.R_L_I * p_body + state.T_L_I) + state.pos_end);
                        // pointBodyToWorld(&point_body, &point_world);
                        point_world.x = p_global(0);
                        point_world.y = p_global(1);
                        point_world.z = p_global(2);
                        point_world.intensity = point_body.intensity;

                        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

                        auto &points_near = Nearest_Points[i];

                        if (iterCount == 0 || rematch_en)
                        {
                            /** Find the closest surfaces in the map **/
                            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

                            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                                     : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                  : true;
                        }

                        if (point_selected_surf[i] == false)
                            continue;

                        VF(4)
                        pabcd;
                        point_selected_surf[i] = false;
                        if (esti_plane(pabcd, points_near, 0.1f))
                        {
                            // loss fuction
                            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); // 公式12）    点到面的距离
                            // if(fabs(pd2) > 0.1) continue;
                            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                            if ((s > 0.9))
                            {
                                point_selected_surf[i] = true;
                                coeffSel_tmpt->points[i].x = pabcd(0);
                                coeffSel_tmpt->points[i].y = pabcd(1);
                                coeffSel_tmpt->points[i].z = pabcd(2);
                                coeffSel_tmpt->points[i].intensity = pd2;

                                res_last[i] = std::abs(pd2);
                            }
                        }
                    }

                    double total_residual = 0.0;
                    effct_feat_num = 0;

                    for (int i = 0; i < coeffSel_tmpt->points.size(); i++)
                    {
                        if (point_selected_surf[i] && (res_last[i] <= 2.0))
                        {
                            laserCloudOri->push_back(feats_down->points[i]);
                            coeffSel->push_back(coeffSel_tmpt->points[i]);
                            total_residual += res_last[i];
                            effct_feat_num++;
                        }
                    }

                    if (effct_feat_num < 10)
                    {
                        // TODO: do something
                        ROS_WARN("No Enough Effective points! May catch error.\n");
                    }

                    res_mean_last = total_residual / effct_feat_num;
                    std::cout << ANSI_COLOR_GREEN << "[-- mapping --]: Effective feature num: " << effct_feat_num
                              << " res_mean_last " << res_mean_last << ANSI_COLOR_RESET << std::endl;

                    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                    Eigen::MatrixXd Hsub(effct_feat_num, 12);
                    Eigen::VectorXd meas_vec(effct_feat_num);
                    Hsub.setZero();

                    // omp_set_num_threads(4);
                    // #pragma omp parallel for
                    for (int i = 0; i < effct_feat_num; i++)
                    {
                        const PointType &laser_p = laserCloudOri->points[i];
                        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
                        M3D point_be_crossmat;
                        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
                        V3D point_this = state.R_L_I * point_this_be + state.T_L_I;
                        M3D point_crossmat;
                        point_crossmat << SKEW_SYM_MATRX(point_this);

                        /*** get the normal vector of closest surface/corner ***/
                        const PointType &norm_p = coeffSel->points[i];
                        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

                        /*** calculate the Measuremnt Jacobian matrix H ***/
                        //  FIXME:
                        //  注意，这里A是列向量，而论文推导种Hj是行向量，故这里从列向量到行向量，做了一次转置 ref:https://github.com/hku-mars/FAST_LIO/issues/62
                        //  而在进行转置处理后，就不会有负号了 ref:https://github.com/hku-mars/FAST_LIO/issues/28
                        V3D C(state.rot_end.transpose() * norm_vec);
                        V3D A(point_crossmat * C);
                        if (extrinsic_est_en)
                        {
                            V3D B(point_be_crossmat * state.R_L_I.transpose() * C);
                            Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
                        }
                        else
                            Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

                        /*** Measuremnt: distance to the closest surface/corner ***/
                        meas_vec(i) = -norm_p.intensity;
                    }

                    Eigen::Vector3d rot_add, t_add;
                    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
                    Eigen::MatrixXd K(DIM_OF_STATES, effct_feat_num);

                    /*** Iterative Kalman Filter Update ***/
                    if (!flg_EKF_inited)
                    {
                        /*** only run in initialization period ***/
                        Eigen::MatrixXd H_init(Eigen::Matrix<double, 15, DIM_OF_STATES>::Zero());
                        Eigen::MatrixXd z_init(Eigen::Matrix<double, 15, 1>::Zero());
                        H_init.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
                        H_init.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
                        z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
                        z_init.block<3, 1>(3, 0) = -state.pos_end;
                        z_init.block<3, 1>(6, 0) = -Log(state.R_L_I);
                        z_init.block<3, 1>(9, 0) = -state.T_L_I;

                        auto H_init_T = H_init.transpose();
                        auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + 0.0001 * Eigen::Matrix<double, 12, 12>::Identity()).inverse(); //  公式20）
                        solution = K_init * z_init;

                        solution.block<12, 1>(0, 0).setZero();
                        state += solution;
                        state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * state.cov; //  公式19）
                    }
                    else
                    {
                        auto &&Hsub_T = Hsub.transpose();
                        H_T_H.block<12, 12>(0, 0) = Hsub_T * Hsub;
                        //  公式(20)上下同时化简R可得
                        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
                            (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse(); //  由于laser point互相独立，R是对角矩阵，即这里的 LASER_POINT_COV
                        K = K_1.block<DIM_OF_STATES, 12>(0, 0) * Hsub_T;                 //  公式20）  ESKF 卡尔曼增益  ---  [3]
 //  如下公式可行，并且效果一致，计算量很大
                        if (0)
                        { //    algo 1
                            auto R = Eigen::MatrixXd::Identity(effct_feat_num, effct_feat_num) * LASER_POINT_COV;
                            auto R_inv = Eigen::MatrixXd::Identity(effct_feat_num, effct_feat_num) * (1.0 / LASER_POINT_COV);
                            // auto K_in = (Hsub_T * R.inverse() * Hsub + state.cov.inverse()).inverse() * Hsub_T * R.inverse();
                            auto K1 = (Hsub_T * R_inv * Hsub + state.cov.inverse()).inverse();
                            auto K_in = K1 * Hsub_T * R_inv;
                        }

						// auto K_in = (H_T_H + state.cov.inverse() * LASER_POINT_COV).inverse() * Hsub_T;
                        // auto K_in = (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse() * Hsub_T;
						
                        // solution = K * meas_vec;
                        // state += solution;

                        auto vec = state_propagat - state;
                        // solution = K * (meas_vec - Hsub * vec.block<6,1>(0,0));
                        // state = state_propagat + solution;

                        solution = K * meas_vec + vec - K * Hsub * vec.block<12, 1>(0, 0);
                        state += solution; //   公式18）  ESKF 更新估计值 --- [4]

                        rot_add = solution.block<3, 1>(0, 0);
                        t_add = solution.block<3, 1>(3, 0);

                        flg_EKF_converged = false;

                        if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015))
                        {
                            flg_EKF_converged = true;
                        }

                        deltaR = rot_add.norm() * 57.3;
                        deltaT = t_add.norm() * 100;

                        // state_propagat = state;
                    }

                    euler_cur = RotMtoEuler(state.rot_end);

                    std::cout << "update: R" << euler_cur.transpose() * 57.3 << " p " << state.pos_end.transpose() << " v " << state.vel_end.transpose() << " bg" << state.bias_g.transpose() << " ba" << state.bias_a.transpose() << std::endl;
                    std::cout << "dR & dT: " << deltaR << " " << deltaT << " res norm:" << res_mean_last << std::endl;

                    /*** Rematch Judgement ***/
                    rematch_en = false;
                    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
                    {
                        rematch_en = true;
                        rematch_num++;
                        std::cout << "rematch_num: " << rematch_num << std::endl;
                    }

                    /*** Convergence Judgements and Covariance Update ***/
                    if (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))
                    {
                        if (flg_EKF_inited)
                        {
                            /*** Covariance Update ***/
                            G.block<DIM_OF_STATES, 12>(0, 0) = K * Hsub;
                            state.cov = (I_STATE - G) * state.cov; //   公式19）  ESKF  估计值和真值的后验协方差   -----  [5]
                            total_distance += (state.pos_end - position_last).norm();
                            position_last = state.pos_end;

                            std::cout << ANSI_COLOR_YELLOW << "position: " << state.pos_end.transpose() << " total distance: " << total_distance
                                      << ", extT: " << state.T_L_I.transpose() << ", extR: " << state.R_L_I << ANSI_COLOR_RESET << std::endl;
                        }

                        break;
                    }
                }

                V3D ext_euler = RotMtoEuler(state.R_L_I);
                fout_pre << std::setw(10) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << state.pos_end.transpose()
                         << " " << ext_euler.transpose() << " " << state.T_L_I.transpose()
                         << " " << state.vel_end.transpose() << " " << state.bias_g.transpose()
                         << " " << state.bias_a.transpose() << " " << state.gravity.transpose()
                         << " " << feats_undistort->points.size() << std::endl;

                if (save_tum_traj)
                {
                    static std::fstream fout_traj(DEBUG_FILE_DIR("odom_trajectory_TUM.txt"), std::ios::out);
                    static std::ostringstream stamp;
                    stamp.str("");
                    if (fout_traj.is_open())
                    {
                        // std::string tstamp = to_string(ros::Time().fromSec(laser_odometry->time));
                        std::string tstamp = std::to_string(Measures.lidar_beg_time);
                        Eigen::Quaterniond q_curr = Eigen::Quaterniond(state.rot_end); // 旋转矩阵转为四元数
                        q_curr.normalize();                                            // 转为四元数之后，需要进行归一化
                        saveTrajectoryTUMformat(fout_traj, tstamp, state.pos_end, q_curr);
                    }
                }
                std::cout << ANSI_COLOR_GREEN_BOLD << "[ mapping ]: iteration count: " << iterCount + 1 << ANSI_COLOR_RESET << std::endl;

                /*** add new frame points to map ikdtree ***/

                map_incremental();

                {
                    PointVector().swap(ikdtree.PCL_Storage);
                    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;
                }
            }

            /******* Publish current frame points in world coordinates:  *******/
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? (*feats_undistort) : (*feats_down);

            int laserCloudFullResNum = laserCloudFullRes2->points.size();

            pcl::PointXYZI temp_point;
            laserCloudFullResColor->clear();
            {
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    RGBpointBodyToWorld(&laserCloudFullRes2->points[i], &temp_point);
                    laserCloudFullResColor->push_back(temp_point);
                }

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(Measures.lidar_end_time); // ros::Time::now(); //.fromSec(last_timestamp_lidar);
                laserCloudFullRes3.header.frame_id = "/camera_init";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }

            /******* Publish Effective points *******/
            {
                laserCloudFullResColor->clear();
                pcl::PointXYZI temp_point;
                for (int i = 0; i < effct_feat_num; i++)
                {
                    RGBpointBodyToWorld(&laserCloudOri->points[i], &temp_point);
                    laserCloudFullResColor->push_back(temp_point);
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(Measures.lidar_end_time);
                laserCloudFullRes3.header.frame_id = "/camera_init";
                pubLaserCloudEffect.publish(laserCloudFullRes3);
            }

            /******* Publish Maps:  *******/
            sensor_msgs::PointCloud2 laserCloudMap;
            pcl::toROSMsg(*featsFromMap, laserCloudMap);
            laserCloudMap.header.stamp = ros::Time().fromSec(Measures.lidar_end_time);
            ;
            laserCloudMap.header.frame_id = "/camera_init";
            pubLaserCloudMap.publish(laserCloudMap);

            /******* Publish Odometry ******/
            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
            odomAftMapped.header.frame_id = "/camera_init";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time().fromSec(Measures.lidar_end_time);
            odomAftMapped.pose.pose.orientation.x = geoQuat.x;
            odomAftMapped.pose.pose.orientation.y = geoQuat.y;
            odomAftMapped.pose.pose.orientation.z = geoQuat.z;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = state.pos_end(0);
            odomAftMapped.pose.pose.position.y = state.pos_end(1);
            odomAftMapped.pose.pose.position.z = state.pos_end(2);

            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                            odomAftMapped.pose.pose.position.y,
                                            odomAftMapped.pose.pose.position.z));
            q.setW(odomAftMapped.pose.pose.orientation.w);
            q.setX(odomAftMapped.pose.pose.orientation.x);
            q.setY(odomAftMapped.pose.pose.orientation.y);
            q.setZ(odomAftMapped.pose.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

            msg_body_pose.header.stamp = ros::Time().fromSec(Measures.lidar_end_time);
            msg_body_pose.header.frame_id = "/camera_odom_frame";
            msg_body_pose.pose.position.x = state.pos_end(0);
            msg_body_pose.pose.position.y = state.pos_end(1);
            msg_body_pose.pose.position.z = state.pos_end(2);
            msg_body_pose.pose.orientation.x = geoQuat.x;
            msg_body_pose.pose.orientation.y = geoQuat.y;
            msg_body_pose.pose.orientation.z = geoQuat.z;
            msg_body_pose.pose.orientation.w = geoQuat.w;

            /******* Publish Path ********/
            msg_body_pose.header.frame_id = "/camera_init";
            path.poses.push_back(msg_body_pose);
            pubPath.publish(path);
        }
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
