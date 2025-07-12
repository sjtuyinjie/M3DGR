
#pragma once
#include <map>
#include <vector>
#include <mutex>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "LocalCartesian.hpp"

class MultisensorOptimization
{
public:
    MultisensorOptimization();
    ~MultisensorOptimization();

    void inputGPS(double t, double latitude, double longtitude, double altitude, double latCov, double lonCov, double altCov);
    void inputOdom(double t, Eigen::Vector3d odomP, Eigen::Quaterniond odomQ);
    void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
    nav_msgs::Path globalPath;
    nav_msgs::Path gpsPath;

private:
    void GPS2XYZ(double latitude, double longtitude, double altitude, double* xyz);
    void optimize();
    void updateGlobalPath();

    std::map<double, std::vector<double>> localPoseMap;
    std::map<double, std::vector<double>> globalPoseMap;
    std::map<double, std::vector<double>> GPSPositionMap;
    bool initGPS;
    bool newGPS;
    
    GeographicLib::LocalCartesian geoConverter;
    std::mutex mPoseMap;
    Eigen::Matrix4d WGPS_T_WVIO;
    Eigen::Vector3d lastP;
    Eigen::Quaterniond lastQ;
    std::thread threadOpt;

    int windowLength;
    Eigen::Vector3d lastLocalP;
};