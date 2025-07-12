#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <random>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "tic_toc.h"
#include "cloud_process.hpp"
#include "voxel_map_util.hpp"
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <ros/package.h>
#include <livox_ros_driver/CustomMsg.h>

#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0, half_map_size = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<vector<KD_TREE<PointType>::VoxelInfoPtr>> Nearest_Nodes;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
PointCloudXYZI::Ptr cloud_with_normal;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

//KD_TREE<ikdTree_PointType> ikdtree;
KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);//T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);//R lidar to imu (imu = r * lidar)
double lidar_time_offset = 0.0;
// record begin time for pose interpolation
deque<double> timestamps_lidar;
nav_msgs::Path path_target_begin, path_target_end;


/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;

//Eigen::VectorXf body_keypose_last(6); //rpy xyz
state_ikfom state_body_last;
vect3 pos_lid;
geometry_msgs::PoseStamped msg_target_pose;
bool have_keyframe = false;
bool save_ikdtree_map;
bool save_final_map;
int num_surfel = 0;

nav_msgs::Path path;
nav_msgs::Path path_target;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// parameters for normal estimation
int N_SCAN, Horizon_SCAN;
string ring_table_dir;
bool check_normal;

// surfel parameters
float planarity = 1.0, mid2min = 100.0;
int surfel_points_min = 20;
int surfel_points_max = 100;
float angle_threshold = 10.0;
bool cloud_surfel = true;
bool point_surfel = true;
int num_cloud_surfel = 0, num_point_surfel = 0;

//voxel map
std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
int max_cov_points_size = 50;
int max_points_size = 50;
double sigma_num = 2.0;
double max_voxel_size = 1.0;
std::vector<int> layer_size;
double min_eigen_value = 0.003;
int max_layer = 0;
std::vector<double> layer_point_size;
bool voxel_hash_en = true;

// for ground truth, target in IMU frame
vector<double>       gt_extrinT(3, 0.0);
vector<double>       gt_extrinR(9, 0.0);
V3D gt_T_wrt_IMU(Zero3d);
M3D gt_R_wrt_IMU(Eye3d);

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    //world <-- imu <-- lidar
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointNormalBodyToWorld(PointType const * const pi, PointType * const po)
{
    pointBodyToWorld(pi, po);

    // normal
    V3F normal_global(state_point.rot.cast<float>() * state_point.offset_R_L_I.cast<float>() * pi->getNormalVector3fMap());
    po->normal_x = normal_global(0);
    po->normal_y = normal_global(1);
    po->normal_z = normal_global(2);
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    // normal
    V3D normal_body(pi->normal_x, pi->normal_y, pi->normal_z); //lidar系下normal
    V3D normal_global(state_point.rot * state_point.offset_R_L_I * normal_body); //w系下坐标
    po->normal_x = normal_global(0);
    po->normal_y = normal_global(1);
    po->normal_z = normal_global(2);
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    

    V3D pos_LiD = pos_lid;

    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }

    if (!need_move) return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

double mean_preprocess = 0.0;
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    double scan_begin_time = msg->header.stamp.toSec() + lidar_time_offset;
    if (scan_begin_time < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(scan_begin_time);
    last_timestamp_lidar = scan_begin_time;
    timestamps_lidar.push_back(last_timestamp_lidar);
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mean_preprocess = mean_preprocess * (scan_count - 1) / scan_count + s_plot11[scan_count] / scan_count;
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    if (runtime_pos_log)
        printf("[ pre-process ]: this time: %0.6f ms, mean : %0.6f ms\n", s_plot11[scan_count] * 1000, mean_preprocess * 1000);
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
   mtx_buffer.lock();
   double preprocess_start_time = omp_get_wtime();
   scan_count ++;
   if (msg->header.stamp.toSec() < last_timestamp_lidar)
   {
       ROS_ERROR("lidar loop back, clear buffer");
       lidar_buffer.clear();
   }
   last_timestamp_lidar = msg->header.stamp.toSec();

   if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
   {
       printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
   }

   if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
   {
       timediff_set_flg = true;
       timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
       printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
   }

   PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
   p_pre->process(msg, ptr);
   lidar_buffer.push_back(ptr);
   time_buffer.push_back(last_timestamp_lidar);

   s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
   mtx_buffer.unlock();
   sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp; //update imu time

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

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
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

bool same_point(PointType a, PointType b){
    return (fabs(a.x-b.x) < EPSS && fabs(a.y-b.y) < EPSS && fabs(a.z-b.z) < EPSS );
}

int process_increments = 0;
void map_incremental()
{
    ROS_INFO("[incremental] start");
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    vector<pair<KD_TREE<PointType>::KD_TREE_NODE*, V3F>> roots_need_update_normal;

    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointNormalBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        PointType & point_world = feats_down_world->points[i];

            ikdtree.updateVoxelInfo(point_world);

        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(point_world.x / filter_size_map_min)*filter_size_map_min + half_map_size;
            mid_point.y = floor(point_world.y / filter_size_map_min)*filter_size_map_min + half_map_size;
            mid_point.z = floor(point_world.z / filter_size_map_min)*filter_size_map_min + half_map_size;
            float dist  = calc_dist(point_world, mid_point);//当前点与box中心的距离

            if (fabs(points_near[0].x - mid_point.x) > half_map_size && fabs(points_near[0].y - mid_point.y) > half_map_size && fabs(points_near[0].z - mid_point.z) > half_map_size){
//                initPointIncInfo(point_world);
                PointNoNeedDownsample.push_back(point_world);
                continue;
            }

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
            {
                PointToAdd.push_back(point_world);
            }
        }
        else
        {
//            initPointIncInfo(point_world);
            PointToAdd.push_back(point_world);
        }
    }
    ROS_INFO("[incremental]");

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
    ROS_INFO("kdtree_incremental_time %f", kdtree_incremental_time);
}

const bool var_contrast(pointWithCov &x, pointWithCov &y) {
    return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }

    if (!have_keyframe)
    {
//        V3D euler_body = SO3ToEuler(state_point.rot);
//        body_keypose_last << euler_body(0), euler_body(1), euler_body(2), state_point.pos(0), state_point.pos(1), state_point.pos(2);
        state_body_last = state_point;
        vect3 pos_target;
        pos_target = state_point.pos + state_point.rot * gt_T_wrt_IMU;
        msg_target_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
        msg_target_pose.header.frame_id = "camera_init";
        msg_target_pose.pose.position.x = pos_target(0);
        msg_target_pose.pose.position.y = pos_target(1);
        msg_target_pose.pose.position.z = pos_target(2);
        Eigen::Quaterniond quat_target(state_point.rot * gt_R_wrt_IMU);
        msg_target_pose.pose.orientation.x = quat_target.x();
        msg_target_pose.pose.orientation.y = quat_target.y();
        msg_target_pose.pose.orientation.z = quat_target.z();
        msg_target_pose.pose.orientation.w = quat_target.w();
        path_target.poses.push_back(msg_target_pose);
        have_keyframe = true;
        return;
    }

    Eigen::Quaterniond q_last(state_body_last.rot.coeffs()[3], state_body_last.rot.coeffs()[0], state_body_last.rot.coeffs()[1], state_body_last.rot.coeffs()[2]);
    Eigen::Quaterniond q_curr(state_point.rot.coeffs()[3], state_point.rot.coeffs()[0], state_point.rot.coeffs()[1], state_point.rot.coeffs()[2]);
    Eigen::Matrix3d r_incre(q_last * q_curr.inverse());
    Eigen::Vector3d rpy = r_incre.eulerAngles(2, 1, 0);

    float x, y, z, roll, pitch, yaw;
    x = state_body_last.pos(0) - state_point.pos(0);
    y = state_body_last.pos(1) - state_point.pos(1);
    z = state_body_last.pos(2) - state_point.pos(2);
    roll = rpy(2);
    pitch = rpy(1);
    yaw = rpy(0);
    float surroundingkeyframeAddingAngleThreshold = 0.2;
    float surroundingkeyframeAddingDistThreshold = 1.0;
    if (abs(roll)  >= surroundingkeyframeAddingAngleThreshold ||
        abs(pitch) >= surroundingkeyframeAddingAngleThreshold ||
        abs(yaw)   >= surroundingkeyframeAddingAngleThreshold ||
        sqrt(x*x + y*y + z*z) >= surroundingkeyframeAddingDistThreshold)
//    if (jjj % 20 == 0)
    {
        vect3 pos_target;
        pos_target = state_point.pos + state_point.rot * gt_T_wrt_IMU;
        Eigen::Quaterniond quat_target(state_point.rot * gt_R_wrt_IMU);
//        Eigen::Quaterniond quat_target(T.block<3, 3>(0, 0));

        msg_target_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
        msg_target_pose.header.frame_id = "camera_init";
        msg_target_pose.pose.position.x = pos_target(0);
        msg_target_pose.pose.position.y = pos_target(1);
        msg_target_pose.pose.position.z = pos_target(2);
        msg_target_pose.pose.orientation.x = quat_target.x();
        msg_target_pose.pose.orientation.y = quat_target.y();
        msg_target_pose.pose.orientation.z = quat_target.z();
        msg_target_pose.pose.orientation.w = quat_target.w();
        path_target_end.poses.push_back(msg_target_pose);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
//    ROS_INFO("[h_share_model]");
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0;
    V3F body(s.pos.x(), s.pos.y(), s.pos.z());

    int num_same_nearest = 0, num_neighbor_nearest = 0, num_diff_nearest = 0;
    int num_node_faraway = 0;
    double time_tree_search = 0, time_voxel_search = 0, time_cov2w = 0;
    int num_valid_voxel_neighbors = 0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointNormalBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));//point: world <-- imu <-- lidar

        const PointType &point_body  = feats_down_body->points[i];
        const PointType &point_world = feats_down_world->points[i];

        V3F body2point = (point_world.getVector3fMap() - body).normalized(); //normalized vector

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];
        auto &voxels_near = Nearest_Nodes[i];
        float angle = 60.0 / 180.0 * M_PI;
//        VF(4) pabcd_surfel; // plane: n * p + d = 0
//        bool surfel_valid = false;
//        bool voxel_search_valid = false;
//        bool tree_search_valid = false;
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
//            PointVector voxel_Nearest_Points;
//            vector<KD_TREE<PointType>::KD_TREE_NODE*> voxel_Nearest_Nodes;
            bool voxel_searching_valid = false;
//            ikdtree.Voxel_Search(point_world, NUM_MATCH_POINTS, voxel_Nearest_Points, voxel_Nearest_Nodes);
//            if (voxel_hash_en)
//            {
//                TicToc t_voxel;
//                //todo fix bugs
//                ikdtree.Voxel_Search_test(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, voxels_near);
//                time_voxel_search += t_voxel.toc();
////                ROS_INFO("#%d hash done", i);
//
////                // point to voxels association (surfel association)
////                VF(4) pabcd_surfel; // plane: n * p + d = 0
////                bool surfel_valid = mergeVoxelCheck(voxels_near, body2point, point_world, pabcd_surfel, 0.15f);
////                if (!surfel_valid)
////                    surfel_valid = singleVoxelCheck(voxels_near, body2point, point_world, pabcd_surfel, 0.2f);
////                /// !!! pabcd_surfel(3) record point to surfel distance
//
////                voxel_searching_valid = surfel_valid;
//                if (voxels_near.size() < NUM_MATCH_POINTS) {
//                    voxel_searching_valid = false;
//                }
//                else {
//                    voxel_searching_valid = true;
//                    ++num_valid_voxel_neighbors;
//                }
//            }

            if (!voxel_searching_valid) {
                TicToc t_tree;
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, voxels_near);
                time_tree_search += t_tree.toc();
            }

            bool planeValid = true;
            if (points_near.size() < NUM_MATCH_POINTS || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
                planeValid = false;
            if (planeValid && check_normal) //check normal direction
            {
                float sum_diff_angle = 0; // angle between normals
                int num_fail_normal = 0;
                for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                    //check map points normal direction, to ensure lidar can see it
                    if (!voxels_near[j])
                        continue;

                    Eigen::Vector3f voxel_normal = voxels_near[j]->normal;

                    if (voxel_normal.dot(body2point) > 0) {
//                        point_selected_surf[i] = false;
//                        planeValid = false;
                        ++num_fail_normal;
                        continue;
                    }
                    sum_diff_angle += acos(point_world.getNormalVector3fMap().dot(voxel_normal));
                }

                // point normal and map surface normal are not consistent
                if (num_fail_normal >= NUM_MATCH_POINTS - 1 || sum_diff_angle / (float)(NUM_MATCH_POINTS - num_fail_normal) > angle )
//                if (num_fail_normal >= NUM_MATCH_POINTS - 1)
                    planeValid = false;
            }
            point_selected_surf[i] = planeValid;
        }

        if (!point_selected_surf[i]) continue;

        point_selected_surf[i] = false;
        VF(4) pabcd; //[n, d], n * p + d = 0
        bool good_plane_feature = false;

        TicToc t_pca;
        VF(4) pabcd_surfel;
        bool good_surfel_feature = false;
        float p2surfel = 10000;
        V3F centroid;

        if (cloud_surfel)
            good_surfel_feature = cloudSurfel(voxels_near, pabcd_surfel, centroid, 0.15f);
        if (good_surfel_feature) {
//            PointType &point_nearest = nodes_near[0]->point;
            V3F cen2point(point_world.x - centroid(0), point_world.y - centroid(1), point_world.z -centroid(2));
            p2surfel = pabcd_surfel.head(3).dot(cen2point);
            num_cloud_surfel++;
        }

        if (!good_surfel_feature && point_surfel)
        {
            //todo maybe not the nearest voxel, compute point to surfel probability, see VoxelMap
            if (voxels_near[0])
            {
                PointType &point_nearest = points_near[0];
                bool same_voxel = abs(point_nearest.x - point_world.x) <= half_map_size &&
                                  abs(point_nearest.y - point_world.y) <= half_map_size &&
                                  abs(point_nearest.z - point_world.z) <= half_map_size;
                const auto &voxel_info = voxels_near[0];
                // point and nearest neighbor inside the same voxel
                // todo 3 * standard deviation
                if (same_voxel && (voxel_info->num_points < 0 || voxel_info->num_points > surfel_points_min)) {
                    V3F plane_centroid(voxel_info->mean_x, voxel_info->mean_y, voxel_info->mean_z);
                    V3F cen2point(point_world.x - plane_centroid(0), point_world.y - plane_centroid(1),
                                  point_world.z - plane_centroid(2));
                    p2surfel = voxel_info->normal.dot(cen2point);
//                    V3F cen2near(point_nearest.x - plane_centroid(0), point_nearest.y - plane_centroid(1),
//                                 point_nearest.z - plane_centroid(2));
                    if (fabs(p2surfel) < 0.20f) {
                        good_surfel_feature = true;
                        pabcd_surfel.head(3) = voxel_info->normal;
                        pabcd_surfel(3) = p2surfel;
                        ++num_point_surfel;
                    }
                }
            }
        }

//        TicToc t_esti_plane;
        if (good_surfel_feature) {
            pabcd = pabcd_surfel;
            good_plane_feature = true;
        }
        else
            good_plane_feature = esti_plane(pabcd, points_near, 0.1f);
        if (good_plane_feature)
        {
            //plane distance
            V3F p_body = point_body.getVector3fMap();
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (good_surfel_feature) {
//                std::cout<< "surfel normal  :\n" <<pabcd_surfel.head(3).transpose() << " p2surfel = " << p2surfel <<
//                         "\nesti normal:\n" <<pabcd.head(3).transpose()  << " p2plane = " << pd2<< std::endl;
                pd2 = p2surfel;
                s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                pabcd = pabcd_surfel;
            }
            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C); // P(IMU)^ [R(imu <-- w) * normal_w]
        if (extrinsic_est_en)
        {
            // B = lidar_p^ R(L <-- I) * corr_normal_I
            // B = lidar_p^ R(L <-- I) * R(I <-- W) * normal_W
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

void saveTraj(const string & pose_target_file)
{
    //also interpolate pose at lidar begin time
    int begin_time_ptr = 0;
    int begin_time_size = timestamps_lidar.size();
    double begin_time = timestamps_lidar[0];
//        int end_time_ptr_left = 0;
    double end_time_left = path_target_end.poses[0].header.stamp.toSec();
    while (begin_time_ptr < begin_time_size) {
        begin_time = timestamps_lidar[begin_time_ptr];
        if (begin_time > end_time_left)
            break;
        ++begin_time_ptr;
    }

    printf("\n..............Saving path................\n");
    printf("path file: %s\n", pose_target_file.c_str());
    ofstream of_beg(pose_target_file);
    of_beg.setf(ios::fixed, ios::floatfield);
    of_beg.precision(12);
    of_beg<< path_target_end.poses[0].header.stamp.toSec()<< " "
          <<path_target_end.poses[0].pose.position.x<< " "
          <<path_target_end.poses[0].pose.position.y<< " "
          <<path_target_end.poses[0].pose.position.z<< " "
          <<path_target_end.poses[0].pose.orientation.x<< " "
          <<path_target_end.poses[0].pose.orientation.y<< " "
          <<path_target_end.poses[0].pose.orientation.z<< " "
          <<path_target_end.poses[0].pose.orientation.w<< "\n";

    for (int i = 1; i < path_target_end.poses.size(); ++i) {
        double end_time_right = path_target_end.poses[i].header.stamp.toSec();
//                printf("end time left: %f\n", end_time_left);
        while (begin_time_ptr < begin_time_size && timestamps_lidar[begin_time_ptr] < end_time_right) {
            begin_time = timestamps_lidar[begin_time_ptr];
            if (abs(begin_time - end_time_right) < 0.00001 || abs(begin_time - end_time_left) < 0.00001) {
                ++begin_time_ptr;
                continue;
            }
            //interpolate between end time left and right
            double dt_l = begin_time - end_time_left;
            double dt_r = end_time_right - begin_time;
            double dt_l_r = end_time_right - end_time_left;
            double ratio_l = dt_l / dt_l_r;
            double ratio_r = dt_r / dt_l_r;

            const auto &pose_l = path_target_end.poses[i - 1].pose;
            const auto &pose_r = path_target_end.poses[i].pose;

            V3D pos_l(pose_l.position.x, pose_l.position.y, pose_l.position.z);
            V3D pos_r(pose_r.position.x, pose_r.position.y, pose_r.position.z);

            Eigen::Quaterniond q_l(pose_l.orientation.w, pose_l.orientation.x, pose_l.orientation.y,
                                   pose_l.orientation.z);
            Eigen::Quaterniond q_r(pose_r.orientation.w, pose_r.orientation.x, pose_r.orientation.y,
                                   pose_r.orientation.z);

            Eigen::Quaterniond  q_begin_time = q_l.slerp(ratio_l, q_r);
            V3D pos_begin_time = pos_l * ratio_r + pos_r * ratio_l;

            of_beg<< begin_time << " "
                  <<pos_begin_time(0)<< " " <<pos_begin_time(1)<< " " <<pos_begin_time(2)<< " "
                  <<q_begin_time.x()<< " "
                  <<q_begin_time.y()<< " "
                  <<q_begin_time.z()<< " "
                  <<q_begin_time.w()<< "\n";

            ++begin_time_ptr;
        }
//                if (abs(begin_time - end_time_right) < 0.000001)
//                    ++begin_time_ptr;
//                printf("end_time_right: %f\n", end_time_right);

        of_beg<< path_target_end.poses[i].header.stamp.toSec()<< " "
              <<path_target_end.poses[i].pose.position.x<< " "
              <<path_target_end.poses[i].pose.position.y<< " "
              <<path_target_end.poses[i].pose.position.z<< " "
              <<path_target_end.poses[i].pose.orientation.x<< " "
              <<path_target_end.poses[i].pose.orientation.y<< " "
              <<path_target_end.poses[i].pose.orientation.z<< " "
              <<path_target_end.poses[i].pose.orientation.w<< "\n";
        end_time_left = end_time_right;
    }
    of_beg.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("common/lidar_time_offset", lidar_time_offset, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_line", N_SCAN, 16);
    nh.param<int>("preprocess/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("preprocess/Horizon_SCAN", p_pre->Horizon_SCAN, 1800);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<bool>("pcd_save/save_ikdtree_map", save_ikdtree_map, false);
    nh.param<bool>("pcd_save/save_final_map", save_final_map, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    half_map_size = filter_size_map_min * 0.5;

    // surfel
    nh.param<float>("surfel/planarity", planarity, 1.0f);
    nh.param<float>("surfel/mid2min", mid2min, 100.0f);
    nh.param<int>("surfel/surfel_points_min", surfel_points_min, 20);
    nh.param<int>("surfel/surfel_points_max", surfel_points_max, 100);
    nh.param<float>("surfel/angle_threshold", angle_threshold, 10.0);
    nh.param<bool>("surfel/cloud_surfel", cloud_surfel, true);
    nh.param<bool>("surfel/point_surfel", point_surfel, true);
    if (surfel_points_min < 20) surfel_points_min = 20;
    if (surfel_points_max < surfel_points_min * 2) surfel_points_max = surfel_points_min * 2;
    ikdtree.max_voxel_points_size = surfel_points_max;

    // ring Fals Normal Estimation parameters
    nh.param<bool>("normal/compute_table", p_pre->compute_table, false);
    nh.param<bool>("normal/compute_normal", p_pre->compute_normal, false);
    nh.param<bool>("normal/check_normal", check_normal, true);
    nh.param<string>("normal/ring_table_dir", ring_table_dir, "/tmp");
    std::string PROJECT_NAME = "log_lio";
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    ring_table_dir = pkg_path + ring_table_dir;
    p_pre->ring_table_dir = ring_table_dir;
    p_pre->runtime_log = runtime_pos_log;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    p_pre->initNormalEstimator();

    //voxel map
    nh.param<vector<double>>("mapping/layer_point_size", layer_point_size,vector<double>());
    for (int i = 0; i < layer_point_size.size(); i++) {
        layer_size.push_back(layer_point_size[i]);
    }
    nh.param<int>("mapping/max_layer", max_layer, 2);
    nh.param<int>("mapping/max_points_size", max_points_size, 100);
    nh.param<int>("mapping/max_cov_points_size", max_cov_points_size, 100);
    nh.param<double>("mapping/voxel_size", max_voxel_size, 1.0);
    nh.param<double>("mapping/plannar_threshold", min_eigen_value, 0.01);
    nh.param<bool>("mapping/voxel_hash_en", voxel_hash_en, true);
    ikdtree.layer_point_size = layer_size;
    ikdtree.max_layer = max_layer;
    ikdtree.max_points_size = max_points_size;
    ikdtree.max_cov_points_size = max_cov_points_size;
    ikdtree.max_voxel_size = max_voxel_size;
//    ikdtree.half_voxel_size = max_voxel_size * 0.5;
    ikdtree.min_eigen_value = min_eigen_value;

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
//    double aver_time_normal = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));//重复？
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    // for ground truth target
    nh.param<vector<double>>("ground_truth/extrinsic_T", gt_extrinT, vector<double>());
    nh.param<vector<double>>("ground_truth/extrinsic_R", gt_extrinR, vector<double>());
    gt_T_wrt_IMU<<VEC_FROM_ARRAY(gt_extrinT);
    gt_R_wrt_IMU<<MAT_FROM_ARRAY(gt_extrinR);
    FILE *fp_target;
    string pos_target_dir = root_dir + "/Log/target_path.txt";

    /*** ROS subscribe initialization ***/
//    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
//        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
//        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();

        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);

            state_point = kf.get_x();
            auto state_cov = kf.get_P();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            //todo do not downsample the very first scan here but in the tree building
            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body); // cloud in lidar frame
            for (auto & p : feats_down_body->points)
                p.getNormalVector3fMap().normalize();

//            pcl::io::savePCDFile("/tmp/feats_down_body.pcd", *feats_down_body);
            ROS_INFO("feats_undistort: %d, feats_down_body: %d", (int)feats_undistort->size(), (int)feats_down_body->size());

            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                double t_init_map = omp_get_wtime();
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointNormalBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));//point: world <-- imu <-- lidar
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // lidar --> imu
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

//            if(0) // If you need to see map point, change to "if(1)"
            if(save_ikdtree_map) // If you need to see map point, change to "if(1)"
            {
                if (frame_num % 30 == 0)
                {
                    PointVector().swap(ikdtree.PCL_Storage);
                    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;

                    featsFromMap->resize(featsFromMap->points.size());
                    pcl::io::savePCDFile("/tmp/ikdtree_map.pcd", *featsFromMap);
                }
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            Nearest_Nodes.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            num_cloud_surfel = 0; num_point_surfel = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            ROS_INFO("[association]: cloud surfel: %d, point surfel: %d", num_cloud_surfel / NUM_MAX_ITERATIONS, num_point_surfel / NUM_MAX_ITERATIONS);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            ROS_INFO("map_incremental %d surfels: %d", ikdtree.validnum(), num_surfel);
//            pcl::io::savePCDFile("/tmp/feats_down_world.pcd", *feats_down_world);

//            /*** add the points to the voxel map ***/
//            voxel_map_incremental();

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                ROS_INFO("feats_undistort: %d, feats_down_body: %d", (int)feats_undistort->size(), (int)feats_down_body->size());

                ROS_INFO("[association]: cloud sufel: %d, point surfel: %d", num_cloud_surfel / NUM_MAX_ITERATIONS, num_point_surfel / NUM_MAX_ITERATIONS);

                ROS_INFO("[map_incremental] tree valid: %d surfels: %d", ikdtree.validnum(), num_surfel);

                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
//                aver_time_normal = aver_time_normal * (frame_num - 1)/frame_num + t_normal / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Cov + Input Downsample: %0.6f this ICP: %0.6f this map incre: %0.6f ave match: %0.6f ave solve: %0.6f ave total: %0.6f ave icp: %0.6f ave construct H: %0.6f\n",
                       t1-t0, t3-t1, t5-t3, aver_time_match, aver_time_solve, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    //save globalPath
    string pose_target_file = root_dir + "/Log/target_path.txt";
    saveTraj(pose_target_file);

    // save final ikdtree map
    if(save_final_map) // If you need to see map point, change to "if(1)"
    {
        PointVector().swap(ikdtree.PCL_Storage);
        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
        featsFromMap->clear();
        featsFromMap->points = ikdtree.PCL_Storage;
        featsFromMap->resize(featsFromMap->points.size());
        pcl::io::savePCDFile("/tmp/ikdtree_final_map.pcd", *featsFromMap);
    }

    // not test yet
    if (p_pre->compute_table) {
        TicToc t_m;
        printf(".....Computing M inverse....\n");
        p_pre->range_image.computeMInverse();
        printf("Computing M inverse cost: %fms\n", t_m.toc());
        printf(".....Saving range image lookup table....\n");
        p_pre->range_image.saveLookupTable(p_pre->ring_table_dir, "ring" + std::to_string(N_SCAN));
    }

    printf(".....[ikdtree] voxel2node: %d, voxel2info: %d\n", (int)ikdtree.voxel2node.size(), (int)ikdtree.voxel2info.size());
    printf(".....check corresponding....\n");
    ikdtree.checkCorrespondding(ikdtree.Root_Node);
    printf("nodes %d\n", ikdtree.num_nodes);
    printf("node --> voxel null %d --> info null %d\n", ikdtree.num_node2_voxel_null, ikdtree.voxel2info_null);
    printf("node --> voxel valid %d --> info valid %d\n", ikdtree.num_node2voxel_valid, ikdtree.voxel2info_valid);
    printf("node --> info same %d diff %d\n", ikdtree.num_same_info, ikdtree.num_diff_info);
    printf("node point --> voxel same %d\n", ikdtree.num_same_voxel_by_pointAndnode);

    int num_no_points = 0, num_fixed_voxel = 0, num_active_voxel = 0;
    int num_voxels = 0;
    double mean_active_points = 0;
    unsigned int sum_active_points = 0;
    for (auto a : ikdtree.voxel2info) {
        if (a.second->num_points < 0 || a.second->num_points >= surfel_points_max)
            ++num_fixed_voxel;
        else if (a.second->num_points == 0)
            ++num_no_points;
        else {
            ++num_active_voxel;
            sum_active_points += a.second->num_points;
            mean_active_points = mean_active_points * (double)(num_active_voxel - 1) / (double)(num_active_voxel) + (double)a.second->num_points / (double)num_active_voxel;
        }
        ++num_voxels;
    }
    printf("num_no_points %d num_fixed_voxel %d num_active_voxel %d active points sum: %d mean: %f\n", num_no_points, num_fixed_voxel, num_active_voxel, sum_active_points, mean_active_points);

    return 0;
}
