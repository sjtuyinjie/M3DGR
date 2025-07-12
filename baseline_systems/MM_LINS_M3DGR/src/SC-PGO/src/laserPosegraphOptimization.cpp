#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"
#include "mapmanager.hpp"
#include <std_msgs/Bool.h>


using namespace gtsam;

using std::cout;
using std::endl;
int SKIP_similar = 1;
int cnt = 0;
double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; 
double rotaionAccumulated = 1000000.0;     // large value means must add the first given frame.

bool isNowKeyFrame = false;
int sleepmapsize  = 0;

Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero

std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<std::pair<int, int>> scLoopICPBuf; 
std::queue<double> scLoopYaw_diff;

std::mutex mBuf;
std::mutex SCBuf;
std::mutex mKF;
std::mutex mtransFinal;
std::mutex mrecentIdxUpdated;
std::condition_variable condvar_bool;
double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
std::mutex mgtSAMgraphMade;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr priorNoise_o;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustLoopNoise_o;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
double scDistThres, scMaximumRadius;

// NOTE:
mapmanager Mapmanager;
std::pair<std::pair<int, int>, std::pair<int, float>> Mapmergingindex = {{0, 0}, {-1, 0}}; 
std::queue<std::pair<std::pair<int, int>, std::pair<int, float>>> MapmergingindexBuf;
std::queue<std::pair<int, int>> Strengthening_constraints;
Eigen::Affine3f transFinal;                                                                
Eigen::Affine3f pretrans1, pretrans2, pretrans3;  
int SleepmapkeyframeposesNum = 0;                                                        
bool isMapmerging = false;
bool isMapsleeping = false;
bool isMapbuilding = false;  
bool issimilaricp_calculation = true;
bool isposegraph_slam = true;
bool isrviz_view = true;
bool isicp_calculation = true; 

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mapmergind;

std::mutex mtxMapsleeping;
std::mutex mtxMapbuilding;
std::mutex mtxsimilaricp_calculation;
std::mutex mtxicp_calculation;
std::mutex mtxposegraph_slam;
std::mutex mtxMapmerging;
std::mutex mtxView;

int pre_merge_idx;
int cur_merge_idx;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;

double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;
ros::Publisher pubOdomRepubVerifier;

std::string test_pre, test_aft;

void downsamplePointCloud(pcl::PointCloud<PointType>::Ptr &cloud, float leaf_size) {
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud);
}

void IssleepingCallback(const std_msgs::Bool::ConstPtr &msg)
{
    bool condition = msg->data;
    mtxMapsleeping.lock();
    isMapsleeping = condition;
    mtxMapsleeping.unlock();
}
void IsbuildingCallback(const std_msgs::Bool::ConstPtr &msg)
{
    bool condition = msg->data;
    mtxMapbuilding.lock();
    isMapbuilding = condition;
    mtxMapbuilding.unlock();
}

std::string padZeros(int val, int num_digits = 6)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D &p)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
} // Pose6DtoGTSAMPose3


void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
    mBuf.lock();
    odometryBuf.push(_laserOdometry);
    mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
    mBuf.lock();
    fullResBuf.push(_laserCloudFullRes);
    mBuf.unlock();
} // laserCloudFullResHandler

void initNoises(void)
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector priorNoiseVector6_o(6);
    priorNoiseVector6_o << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
    priorNoise_o = noiseModel::Diagonal::Variances(priorNoiseVector6_o);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.0002;         // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    robustLoopNoise_o = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

} // initNoises

Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
} // getOdom

Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr cloudIn, const Pose6D tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<PointType>::Ptr local2globalwithAffine3f(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Eigen::Affine3f &transCur)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

Pose6D posetran(const Pose6D &poseIn, const Eigen::Affine3f &transCur)
{
    Pose6D poseOut;
    Eigen::Affine3f poseIn_a = pcl::getTransformation(poseIn.x, poseIn.y, poseIn.z, poseIn.roll, poseIn.pitch, poseIn.yaw);
    Eigen::Affine3f poseout_a = transCur * poseIn_a;
    float tx, ty, tz, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(poseout_a, tx, ty, tz, roll, pitch, yaw);
    poseOut.x = tx;
    poseOut.y = ty;
    poseOut.z = tz;
    poseOut.roll = roll;
    poseOut.pitch = pitch;
    poseOut.yaw = yaw;
    return poseOut;
}

void pubPath(void)
{
    // pub odom and path
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    mKF.lock();
    for (int node_idx = 0; node_idx < int(Mapmanager.ActiveMap->keyframeposesUpdated.size()) - 1; ++node_idx) 
    {
        const Pose6D &pose_est = Mapmanager.ActiveMap->keyframeposesUpdated.at(node_idx); // upodated poses

        nav_msgs::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = ros::Time().fromSec(Mapmanager.ActiveMap->keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock();
    pubOdomAftPGO.publish(odomAftPGO); // last pose
    pubPathAftPGO.publish(pathAftPGO); // poses

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, "camera_init", "/aft_pgo"));
} // pubPath

void updatePoses(void)
{
    mKF.lock();
    for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size()); ++node_idx)
    {
        Pose6D &p = Mapmanager.ActiveMap->keyframeposesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mrecentIdxUpdated.lock();
    recentIdxUpdated = int(Mapmanager.ActiveMap->keyframeposesUpdated.size()) - 1;
    mrecentIdxUpdated.unlock();
    mKF.unlock();
} // updatePoses

void runISAM2opt(void)
{
    mtxPosegraph.lock();
    // called when a variable added
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    isam->update();
    isam->update();
    isam->update();

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    mtxPosegraph.unlock();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
        transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw());

    int numberOfCores = 8;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom->x + transCur(0, 1) * pointFrom->y + transCur(0, 2) * pointFrom->z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom->x + transCur(1, 1) * pointFrom->y + transCur(1, 2) * pointFrom->z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom->x + transCur(2, 1) * pointFrom->y + transCur(2, 2) * pointFrom->z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void FindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr nearKeyframes, const int key, const int submap_size)
{
    // extract and stacking near keyframes (in global coord) 
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i)
    {
        int keyNear = key + i;
        mKF.lock();
        if (keyNear < 0 || keyNear >= int(Mapmanager.ActiveMap->keyframeClouds.size()))
        {
            mKF.unlock();
            continue;
        }
        else
        {
            int size = int(Mapmanager.ActiveMap->keyframeClouds[keyNear]->size());
            mKF.unlock();
            if (size < 0)
            {
                continue;
            }
        }

        mKF.lock();
        *nearKeyframes += *local2global(Mapmanager.ActiveMap->keyframeClouds[keyNear], Mapmanager.ActiveMap->keyframeposesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty()){
        return;
    }
    // downsample near keyframes
    // mtxICP.lock();
    // downSizeFilterICP.setInputCloud(nearKeyframes);
    // downSizeFilterICP.filter(*nearKeyframes);
    // mtxICP.unlock();

} 

void sleepmapFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &submap_size, const int &sleepmap_idx)
{
    // extract and stacking near keyframes (in global coord) 
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i)
    {
        int keyNear = key + i;
        mKF.lock();
        if (keyNear < 0 || keyNear >= int(Mapmanager.SleepMapList[sleepmap_idx]->keyframeClouds.size())){
            mKF.unlock();
            continue;}
        *nearKeyframes += *local2global(Mapmanager.SleepMapList[sleepmap_idx]->keyframeClouds[keyNear], Mapmanager.SleepMapList[sleepmap_idx]->keyframeposesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;
    // downsample near keyframes
    // pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    // mtxICP.lock();
    // downSizeFilterICP.setInputCloud(nearKeyframes);
    // downSizeFilterICP.filter(*cloud_temp);
    // mtxICP.unlock();
    // *nearKeyframes = *cloud_temp;
}

void activemapFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &submap_size)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i)
    {
        int keyNear = key + i;
        mKF.lock();
        if (keyNear < 0 || keyNear >= int(Mapmanager.ActiveMap->keyframeClouds.size()))
        {   mKF.unlock();
            continue;}
    
        *nearKeyframes += *local2global(Mapmanager.ActiveMap->keyframeClouds[keyNear], Mapmanager.ActiveMap->keyframeposesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;
    // downsample near keyframes
    // pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    // mtxICP.lock();
    // downSizeFilterICP.setInputCloud(nearKeyframes);
    // downSizeFilterICP.filter(*cloud_temp);
    // mtxICP.unlock();
    // *nearKeyframes = *cloud_temp;
}

std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx)
{
    int historyKeyframeSearchNum = 10; 
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());

    FindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 2);
    FindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum);


    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg); 

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(10); 
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(2);

    // Align pointclouds 
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    float loopFitnessScoreThreshold = 0.7; 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    }
    else
    {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    mKF.lock();
    Eigen::Affine3f tWrong = pcl::getTransformation(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].x, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].y, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].z, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].roll, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].pitch, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].yaw);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated[_loop_kf_idx]);
    mKF.unlock();
    return poseFrom.between(poseTo);
} 

std::optional<gtsam::Pose3> doICPsimliarOpt(int _loop_kf_idx, int _curr_kf_idx)
{
    int historyKeyframeSearchNum = 10; 
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    FindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 2);
    FindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100); 
    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
    float loopFitnessScoreThreshold = 2.0; // user parameter but fixed low value is safe.
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        // std::cout << "[Loop after merging maps] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    }
    else
    {
        // std::cout << "[Loop after merging maps] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    mKF.lock();
    Eigen::Affine3f tWrong = pcl::getTransformation(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].x, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].y, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].z, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].roll, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].pitch, 
                                                                                Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].yaw);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated[_loop_kf_idx]);
    mKF.unlock();
    return poseFrom.between(poseTo);
} 

bool doICPslmilarmatch(int sleepmapindex, int sleepind, int activeind, Eigen::Affine3f pre)
{
    int historyKeyframeSearchNum = 15;
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    activemapFindNearKeyframesCloud(cureKeyframeCloud, activeind, 2); 
    sleepmapFindNearKeyframesCloud(targetKeyframeCloud, sleepind, historyKeyframeSearchNum, sleepmapindex);
    *cureKeyframeCloud = *local2globalwithAffine3f(cureKeyframeCloud, pre);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(200); 
    icp.setMaximumIterations(100);         
    icp.setTransformationEpsilon(1e-6);    
    icp.setEuclideanFitnessEpsilon(1e-6);  
    icp.setRANSACIterations(2);            

    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    float loopFitnessScoreThreshold = 0.6;
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        cout << "Don't hold back, it's so embarrassing!" << "*******" <<(icp.getFitnessScore()) <<endl;
        return false;
    }
    else
    {
        cout << "Hold back, this is great!" << "*******" <<(icp.getFitnessScore()) <<endl;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        mtransFinal.lock();
        transFinal =  correctionLidarFrame * pre;
        mtransFinal.unlock();
        return true;
    }
} 

void performSimilarClosure(void)
{
    mKF.lock();
    if (Mapmanager.SleepMapList.empty() || Mapmanager.ActiveMap->scmanager.polarcontext_invkeys_mat_.size() < 2)
        {mKF.unlock();
        return;}
    mKF.unlock();
    std::vector<double> similar_mapindexes;
    std::vector<std::pair<int, float>> detectResult_sleepmap;
    mKF.lock();
    for (int i = 0; i < Mapmanager.SleepMapList.size(); ++i)
    {
        detectResult_sleepmap.push_back(Mapmanager.SleepMapList[i]->scmanager.similar_detectLoopClosureID(Mapmanager.ActiveMap->scmanager.polarcontext_invkeys_mat_, Mapmanager.ActiveMap->scmanager.polarcontexts_));
        similar_mapindexes.push_back(Mapmanager.SleepMapList[i]->optimal_matchvalue);
    }
    mKF.unlock();
    int min_index = 0;
    int min_value = similar_mapindexes[0];
    for (int i = 1; i < similar_mapindexes.size(); ++i)
    {
        if (similar_mapindexes[i] < min_value)
        {
            min_value = similar_mapindexes[i];
            min_index = i;
        }
    }
    mKF.lock();
    int activeind = Mapmanager.ActiveMap->scmanager.polarcontexts_.size() - 1;
    mKF.unlock();
    auto detectResult = detectResult_sleepmap[min_index];
    if (detectResult.first > 0)
    {
        mKF.lock();
        Mapmanager.Wakeup_Mapindex = min_index;
        mKF.unlock();
        std::pair<int, int> index{min_index, activeind};
        Mapmergingindex.first = index;
        Mapmergingindex.second = detectResult;
        mapmergind.lock();
        MapmergingindexBuf.push(Mapmergingindex);
        mapmergind.unlock();
    }
}

void performSCLoopClosure(void)
{
    mKF.lock();
    if (int(Mapmanager.ActiveMap->keyframeposes.size()) < Mapmanager.ActiveMap->scmanager.NUM_EXCLUDE_RECENT) // do not try too early  
        {mKF.unlock();
        return;}
    auto detectResult = Mapmanager.ActiveMap->scmanager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    mKF.unlock();  
    int SCclosestHistoryFrameID = detectResult.first;                         // SCclosestHistoryFrameID
    float SCyaw_diff = detectResult.second;
    if (SCclosestHistoryFrameID != -1)
    {
        const int prev_node_idx = SCclosestHistoryFrameID;
        mKF.lock();
        const int curr_node_idx = Mapmanager.ActiveMap->keyframeposes.size() - 1;
        mKF.unlock();
        //cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        SCBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx)); // scLoopICPBuf
        scLoopYaw_diff.push(SCyaw_diff);
        SCBuf.unlock();
    }
} 


void process_similaricp(void)
{
    float loopClosureFrequency = 10; // can change
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        mtxsimilaricp_calculation.lock();
        if (issimilaricp_calculation)
        {
            mtxsimilaricp_calculation.unlock();
            rate.sleep();
            while (!MapmergingindexBuf.empty())
            {
                if (MapmergingindexBuf.size() > 30)
                {
                    cout<<"silmiler_num"<<MapmergingindexBuf.size()<<endl;
                    ROS_WARN("Too many silmiler clousre is waiting ... Do process_similar less frequently (adjust loopClosureFrequency)");
                }

                mapmergind.lock();
                std::pair<std::pair<int, int>, std::pair<int, float>> similar_pair = MapmergingindexBuf.front();
                MapmergingindexBuf.pop();
                mapmergind.unlock();
                std::pair<int, int> similar_idx_pair = similar_pair.second; 
                const float rot = similar_idx_pair.second;                     
                const int sleepmap_node_idx = similar_idx_pair.first;          
                const int activepmap_node_idx = similar_pair.first.second; 
                int similarmapindex = similar_pair.first.first;             
                mKF.lock();
                Pose6D t1 = Mapmanager.SleepMapList[similarmapindex]->keyframeposesUpdated[sleepmap_node_idx];
                Pose6D t2 = Mapmanager.SleepMapList[similarmapindex]->keyframeposesUpdated[(Mapmanager.SleepMapList[similarmapindex]->keyframeposesUpdated.size()-1)];
                mKF.unlock();
                Eigen::Affine3f sc_initial = pcl::getTransformation(0, 0, 0, 0, 0, rot);
                Eigen::Affine3f pretrans_temp = pcl::getTransformation(t2.x, t2.y, t2.z, t2.roll, t2.pitch, 0);
                pretrans3 = sc_initial * pretrans_temp;
                pretrans2 = pcl::getTransformation(t2.x, t2.y, t2.z, t2.roll, t2.pitch, t2.yaw);
                pretrans1 = pcl::getTransformation(t1.x, t1.y, t1.z, t1.roll, t1.pitch, t1.yaw);
                if (doICPslmilarmatch(similarmapindex, sleepmap_node_idx, activepmap_node_idx, pretrans2))
                {
                    mKF.lock();
                    sleepmapsize = Mapmanager.SleepMapList[similarmapindex]->keyframeposes.size();
                    mKF.unlock();
                    pre_merge_idx = sleepmap_node_idx;
                    cur_merge_idx = sleepmapsize + activepmap_node_idx;
                    
                    mtxposegraph_slam.lock();
                    isposegraph_slam = false;
                    mtxposegraph_slam.unlock();

                    mtxsimilaricp_calculation.lock();
                    issimilaricp_calculation = false;
                    mtxsimilaricp_calculation.unlock();

                    mtxicp_calculation.lock();
                    isicp_calculation = false;
                    mtxicp_calculation.unlock();

                    mtxMapmerging.lock();
                    isMapmerging = true;
                    mtxMapmerging.unlock();
                    
                    int n = 0;
                    mapmergind.lock();
                    while (!MapmergingindexBuf.empty())
                    {
                        if(n<=5){
                        std::pair<int, int> map_pair;
                        map_pair.first = MapmergingindexBuf.front().second.first; 
                        map_pair.second = MapmergingindexBuf.front().first.second + sleepmapsize;  
                        Strengthening_constraints.push(map_pair);
                        mapmergind.unlock();
                        }
                        MapmergingindexBuf.pop();
                        mapmergind.unlock();
                        n++;
                    }
                    mapmergind.unlock();
                }
            }
        }
        mtxsimilaricp_calculation.unlock();
    }
}





void process_map_surveillance(void)
{
    // while (1)
    // {
        mtxMapsleeping.lock();
        if (isMapsleeping) 
        {
            std::cout << "Whether to sleep the currently active map: " << isMapsleeping << std::endl;
            isMapsleeping = false;
            mtxMapsleeping.unlock();

            mtxposegraph_slam.lock();
            isposegraph_slam = false;
            mtxposegraph_slam.unlock();

            mtxsimilaricp_calculation.lock();
            issimilaricp_calculation = false;
            mtxsimilaricp_calculation.unlock();

            mtxicp_calculation.lock();
            isicp_calculation = false;
            mtxicp_calculation.unlock();

            mtxView.lock();
            isrviz_view = false;
            mtxView.unlock();

            mKF.lock();
            Mapmanager.act_change();
            mKF.unlock();
            mtransFinal.lock();
            transFinal.setIdentity();
            mtransFinal.unlock();

            SCBuf.lock();
            while (!scLoopICPBuf.empty())
            {
                scLoopICPBuf.pop();
            }
            SCBuf.unlock();
        }
        mtxMapsleeping.unlock();
        mtxMapbuilding.lock();
        if (isMapbuilding) 
        {
            std::cout << "Whether to start a new active map: " << isMapbuilding << std::endl;
            isMapbuilding = false;
            mtxMapbuilding.unlock();

            mtxposegraph_slam.lock();
            isposegraph_slam = true;
            mtxposegraph_slam.unlock();

            mtxsimilaricp_calculation.lock();
            issimilaricp_calculation = true;
            mtxsimilaricp_calculation.unlock();

            mtxicp_calculation.lock();
            isicp_calculation = true;
            mtxicp_calculation.unlock();

            mtxView.lock();
            isrviz_view = true;
            mtxView.unlock();

            mKF.lock();
            Mapmanager.act_reset();
            Mapmanager.ActiveMap->scmanager.setSCdistThres(scDistThres);
            Mapmanager.ActiveMap->scmanager.setMaximumRadius(scMaximumRadius);
            mKF.unlock();
            mtxPosegraph.lock();
            gtsam::ISAM2 *newisam;
            isam = newisam;
            gtsam::ISAM2Params optParameters;
            optParameters.relinearizeThreshold = 0.1;
            optParameters.relinearizeSkip = 1;
            isam = new ISAM2(optParameters);
            gtsam::NonlinearFactorGraph newGraphFactors;
            gtSAMgraph = newGraphFactors;   
            gtsam::Values NewGraphValues;   
            initialEstimate = NewGraphValues;
            mtxPosegraph.unlock();
            mgtSAMgraphMade.lock();
            gtSAMgraphMade = false;
            mgtSAMgraphMade.unlock();
            mrecentIdxUpdated.lock();
            recentIdxUpdated = 0;
            mrecentIdxUpdated.unlock();
        }
        mtxMapbuilding.unlock();
        // wait (must required for running the while loop)
    //     std::chrono::milliseconds dura(2);
    //     std::this_thread::sleep_for(dura);
    // }
}

void pubMap(void)
{
    int SKIP_FRAMES = 1; // sparse map visulalization to save computations 
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock();
    for (int node_idx = 0; node_idx < int(Mapmanager.ActiveMap->keyframeposesUpdated.size()); ++node_idx)
    {
        if (counter % SKIP_FRAMES == 0)
        {
            *laserCloudMapPGO += *local2global(Mapmanager.ActiveMap->keyframeClouds[node_idx], Mapmanager.ActiveMap->keyframeposesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock();
    if (!laserCloudMapPGO->empty())
    {
        pcl::PointCloud<PointType>::Ptr thisKeyFrame_fir(new pcl::PointCloud<PointType>());
        *thisKeyFrame_fir = *laserCloudMapPGO;
        // pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        // downSizeFilterGlobalMapKeyFrames.setLeafSize(1, 1, 1); // for global map visualization
        // downSizeFilterGlobalMapKeyFrames.setInputCloud(thisKeyFrame_fir);
        // downSizeFilterGlobalMapKeyFrames.filter(*thisKeyFrame_fir);
    }
    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    pubMapAftPGO.publish(laserCloudMapPGOMsg); 
}

void process_pg()
{
    while (1)
    {
        process_map_surveillance();
        mtxposegraph_slam.lock();
        if (isposegraph_slam || !odometryBuf.empty())
        {
            mtxposegraph_slam.unlock();
            while (!odometryBuf.empty() && !fullResBuf.empty()) 
            {
                mBuf.lock();
                while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                    odometryBuf.pop();
                if (odometryBuf.empty())
                {
                    mBuf.unlock();
                    break;
                }

                timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
                timeLaser = fullResBuf.front()->header.stamp.toSec();

                laserCloudFullRes->clear();
                pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
                
                pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
                fullResBuf.pop();

                Pose6D pose_curr = getOdom(odometryBuf.front());

                odometryBuf.pop();

                mBuf.unlock();
                mtransFinal.lock();
                pose_curr = posetran(pose_curr, transFinal);
                mtransFinal.unlock();
                odom_pose_prev = odom_pose_curr;
                odom_pose_curr = pose_curr;
                Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

                double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z); // note: absolute value.
                translationAccumulated += delta_translation;
                rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.

                if (translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap)
                {
                    isNowKeyFrame = true;
                    translationAccumulated = 0.0; // reset
                    rotaionAccumulated = 0.0;     // reset
                }
                else
                {
                    isNowKeyFrame = false;
                }

                if (!isNowKeyFrame)
                    continue;
                cnt++;
                pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
                downSizeFilterScancontext.setInputCloud(thisKeyFrame);
                downSizeFilterScancontext.filter(*thisKeyFrameDS);
                mKF.lock();
                Mapmanager.addkeyframewithScancontext(thisKeyFrameDS, pose_curr, timeLaserOdometry);
                SleepmapkeyframeposesNum = 0; 
                for (int i = 0; i < Mapmanager.SleepMapList.size(); ++i)
                {
                    SleepmapkeyframeposesNum += Mapmanager.SleepMapList[i]->keyframeposes.size();
                }
                const int prev_node_idx = Mapmanager.ActiveMap->keyframeposes.size() - 2;
                const int curr_node_idx = Mapmanager.ActiveMap->keyframeposes.size() - 1;
                mKF.unlock();
                mgtSAMgraphMade.lock();
                if (!gtSAMgraphMade /* prior node */)
                {
                    const int init_node_idx = 0;
                    mKF.lock();
                    gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposes.at(0));
                    mKF.unlock();
                    mtxPosegraph.lock();
                    {
                        // prior factor
                        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise)); 
                        initialEstimate.insert(init_node_idx, poseOrigin);
                        // runISAM2opt();
                    }
                    mtxPosegraph.unlock();

                    gtSAMgraphMade = true;
                    mgtSAMgraphMade.unlock();
                    // cout << "posegraph prior node " << init_node_idx << " added" << endl; //
                }
                else /* consecutive node (and odom factor) after the prior added */
                {    // == keyframePoses.size() > 1
                    mgtSAMgraphMade.unlock();
                    mKF.lock();
                    gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposes.at(prev_node_idx));
                    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposes.at(curr_node_idx));
                    mKF.unlock();
                    mtxPosegraph.lock();
                    {
                        // odom factor
                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((prev_node_idx), (curr_node_idx), poseFrom.between(poseTo), odomNoise));
                        initialEstimate.insert((curr_node_idx), poseTo);
                    }
                    mtxPosegraph.unlock();
                    if ((curr_node_idx) % 100 == 0)
                        cout << "\033[33m" << "posegraph odom node " << (curr_node_idx) << " added." << endl;
                }
                performSCLoopClosure();
                if(cnt % SKIP_similar == 0){
                    performSimilarClosure();
                }
                mgtSAMgraphMade.lock();
                if (gtSAMgraphMade)
                {
                    mgtSAMgraphMade.unlock();
                    runISAM2opt();
                    //cout << "running isam2 optimization ..." << endl;
                }
                mgtSAMgraphMade.unlock();

            }
        }
        mtxposegraph_slam.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void process_map(void)
{
    while (1)
    {
        while (isMapmerging)
        {
            isMapmerging = false;
            
            mKF.lock();
            {
                mtransFinal.lock();
                Mapmanager.transformmap(transFinal);
                mtransFinal.unlock();
                Mapmanager.mergemap();
                Mapmanager.sleep_delete();
            }
            mKF.unlock();
            gtsam::ISAM2 *newisam;
            mtxPosegraph.lock();
            isam = newisam;
            gtsam::ISAM2Params optParameters;
            optParameters.relinearizeThreshold = 0.01;
            optParameters.relinearizeSkip = 1;
            isam = new ISAM2(optParameters);
            gtsam::NonlinearFactorGraph newGraphFactors;
            gtSAMgraph = newGraphFactors;  
            gtsam::Values NewGraphValues;  
            initialEstimate = NewGraphValues;
            mtxPosegraph.unlock();
            mKF.lock();
            gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(0));
            gtsam::Pose3 poseOrigin_o = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(sleepmapsize));
            mKF.unlock();
            mtxPosegraph.lock();
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseOrigin, priorNoise)); 
            initialEstimate.insert(0, poseOrigin);
            // gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(sleepmapsize, poseOrigin_o, priorNoise_o));
            // initialEstimate.insert(sleepmapsize, poseOrigin_o);
            for(int i = 1; i<sleepmapsize; ++i){
                mKF.lock();
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i-1));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i));
                mKF.unlock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((i-1), (i), poseFrom.between(poseTo), odomNoise));
                initialEstimate.insert((i), poseTo);
            }
            mKF.lock();
            initialEstimate.insert((sleepmapsize),  Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(sleepmapsize)));
            for(int i = (sleepmapsize+1); i<int(Mapmanager.ActiveMap->keyframeposesUpdated.size()); ++i){
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i-1));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i));
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((i-1), (i), poseFrom.between(poseTo), odomNoise));
                initialEstimate.insert((i), poseTo);
            }
            mKF.unlock();
            auto simliarOpt_optional = doICPsimliarOpt(pre_merge_idx, cur_merge_idx);
            if (simliarOpt_optional)
            {
                gtsam::Pose3 relativeOpt_pose = simliarOpt_optional.value();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cur_merge_idx, pre_merge_idx,  relativeOpt_pose, robustLoopNoise_o));
            }
            while(!Strengthening_constraints.empty()){
                cout<<"PGO:"<<Strengthening_constraints.size()<<endl;
                auto simliarOpt_additional= doICPsimliarOpt(Strengthening_constraints.front().first, Strengthening_constraints.front().second);
                if (simliarOpt_additional)
                {
                    gtsam::Pose3 relativeOpt_poseadditional = simliarOpt_additional.value();
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((Strengthening_constraints.front().second), 
                                                                                            ((Strengthening_constraints.front().first)),  
                                                                                            relativeOpt_poseadditional, 
                                                                                            robustLoopNoise_o));
                }
                Strengthening_constraints.pop();
            }
            
            mtxPosegraph.unlock();
            
            mtxposegraph_slam.lock();
            isposegraph_slam = true;
            mtxposegraph_slam.unlock();
            mtxsimilaricp_calculation.lock();
            issimilaricp_calculation = true;
            mtxsimilaricp_calculation.unlock();
            mtxicp_calculation.lock();
            isicp_calculation = true;
            mtxicp_calculation.unlock();
            
            SleepmapkeyframeposesNum = 0;
            mKF.lock();
            cout << "Number of sleeping maps after fusion: " << Mapmanager.SleepMapList.size() << endl;
            cout << "The number of keyframes of the currently active map after fusion: " << Mapmanager.ActiveMap->keyframeposes.size() << endl;
            mKF.unlock();
        }
        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void process_icp(void)
{
    float loopClosureFrequency = 1; // can change
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        if (isicp_calculation)
        {
            rate.sleep();
            SCBuf.lock();
            while (!scLoopICPBuf.empty())
            {
                if (scLoopICPBuf.size() > 30)
                {
                    ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
                }

                mBuf.lock();
                std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
                float Yaw_diff = scLoopYaw_diff.front();
                scLoopYaw_diff.pop();
                scLoopICPBuf.pop();
                mBuf.unlock();
                Eigen::Affine3f sc_initial = pcl::getTransformation(0, 0, 0, 0, 0, Yaw_diff);
                const int prev_node_idx = loop_idx_pair.first;
                const int curr_node_idx = loop_idx_pair.second;

                auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
                if (relative_pose_optional)
                {
                    gtsam::Pose3 relative_pose = relative_pose_optional.value();
                    mtxPosegraph.lock();
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>( curr_node_idx, prev_node_idx, relative_pose, robustLoopNoise));
                    // runISAM2opt();
                    mtxPosegraph.unlock();
                }
            }
            SCBuf.unlock();
        }
    }
} // process_icp

void process_view(void)
{
    float vizmapFrequency = 10; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (ros::ok())
    {   
        mtxView.lock();
        if (isrviz_view)
        {
            mtxView.unlock();
            rate.sleep();
            mrecentIdxUpdated.lock();
            if (recentIdxUpdated > 10)
            {
                mrecentIdxUpdated.unlock();
                pubPath();
                pubMap();
            }
            mrecentIdxUpdated.unlock();
        }
        mtxView.unlock();
    }
} // pointcloud_viz

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserPGO");
    ros::NodeHandle nh;

    nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 0.5); 
    nh.param<double>("keyframe_deg_gap", keyframeDegGap, 5.0);     
    keyframeRadGap = rad2deg_(keyframeDegGap);                    
    nh.param<double>("sc_dist_thres", scDistThres, 0.6);     
    nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();
    transFinal.setIdentity();
    pretrans1.setIdentity();
    pretrans2.setIdentity();
    pretrans3.setIdentity();

    Mapmanager.ActiveMap->scmanager.setSCdistThres(scDistThres);
    Mapmanager.ActiveMap->scmanager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4;
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
    nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);

    ros::Subscriber subissleep = nh.subscribe<std_msgs::Bool>("Issleeping", 10, IssleepingCallback);
    ros::Subscriber subisbuild = nh.subscribe<std_msgs::Bool>("Isbuilding", 10, IsbuildingCallback);

    pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
    pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);
    pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
    pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);

    pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);
    std::thread posegraph_slam{process_pg}; 
    std::thread icp_calculation{process_icp};
    std::thread similaricp_calculation{process_similaricp};
    std::thread map_merging{process_map};
    std::thread rviz_view{process_view}; 
    // std::thread map_surveillance{process_map_surveillance}; 
    ros::spin();

    return 0;
}
