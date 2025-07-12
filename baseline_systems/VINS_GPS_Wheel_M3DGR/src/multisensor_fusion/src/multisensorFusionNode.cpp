
#include <iostream>
#include <fstream>
#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "optimization/multisensorOpt.h"

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg);
void vio_callback(const nav_msgs::OdometryConstPtr &pose_msg);

MultisensorOptimization multisensorOptimization;
ros::Publisher globalOdomPub, globalPathPub, gpsPathPub;
nav_msgs::Path* globalPath;
nav_msgs::Path* gpsPath;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;

double last_vio_t = -1; // time of last vio frame

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "multisensor_fusion_node");
    ros::NodeHandle nh("~");

    globalPath = &multisensorOptimization.globalPath;
    gpsPath = &multisensorOptimization.gpsPath;

    ros::Subscriber sub_GPS = nh.subscribe("/gps/data_raw", 100, GPS_callback);
    ros::Subscriber sub_vio = nh.subscribe("/vins_estimator/odometry", 100, vio_callback);

    globalPathPub = nh.advertise<nav_msgs::Path>("global_path", 100);
    globalOdomPub = nh.advertise<nav_msgs::Odometry>("global_odometry", 100);
    gpsPathPub = nh.advertise<nav_msgs::Path>("gps_path", 100);

    ros::spin();
    return 0;
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;

    Eigen::Vector3d vio_t;
    vio_t.x() = pose_msg->pose.pose.position.x;
    vio_t.y() = pose_msg->pose.pose.position.y;
    vio_t.z() = pose_msg->pose.pose.position.z;
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    multisensorOptimization.inputOdom(t, vio_t, vio_q);

    m_buf.lock();
    while (!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        // ROS_INFO("last_vio_t: %f, gps_t: %f, Time deviation: %f", last_vio_t, gps_t, last_vio_t - gps_t);

        // Different from VINS-Fusion, which has synchronized timestamps at data-publish stage.
        if (gps_t >= t - 0.1 && gps_t <= t + 0.1)
        {
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            double latVal = GPS_msg->position_covariance[0];
            double lonVal = GPS_msg->position_covariance[4];
            double altVal = GPS_msg->position_covariance[8];

            multisensorOptimization.inputGPS(t, latitude, longitude, altitude, latVal, lonVal, altVal);
            
            gpsQueue.pop();
            break;
        }
        else if (gps_t < t - 0.1)
        {
            gpsQueue.pop();
        }
        else if (gps_t > t + 0.1)
        {
            break;
        }
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen::Quaterniond global_q;
    multisensorOptimization.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.w = global_q.w();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    globalOdomPub.publish(odometry);
    globalPathPub.publish(*globalPath);
    gpsPathPub.publish(*gpsPath);
    
    // write result to csv file
    std::ofstream foutC("/home/xiaoqiang/output/vio_global.csv", std::ios::app);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
          << global_t.y() << ","
          << global_t.z() << ","
          << global_q.w() << ","
          << global_q.x() << ","
          << global_q.y() << ","
          << global_q.z() << std::endl;
    foutC.close();
}