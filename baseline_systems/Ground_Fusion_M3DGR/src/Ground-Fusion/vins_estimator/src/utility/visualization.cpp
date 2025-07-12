/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry,pub_latest_pure_odometry, pub_latest_wheel_odometry, pub_latest_pure_wheel_odometry;
ros::Publisher pub_lines, pub_marg_lines;
ros::Publisher pub_path;
// ros::Publisher pub_groundtruth;
ros::Publisher pub_wheel_preintegration;
ros::Publisher pub_pure_wheel_preintegration;
ros::Publisher pub_imu_preintegration,pub_pure_imu_preintegration;
ros::Publisher pub_wheel_rawodom;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;
nav_msgs::Path groundtruth_path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher pub_image_track;

//gnss new
ros::Publisher pub_gnss_lla;
ros::Publisher pub_enu_path, pub_rtk_enu_path;
nav_msgs::Path enu_path, rtk_enu_path;
ros::Publisher pub_anc_lla;
ros::Publisher pub_enu_pose;
ros::Publisher pub_sat_info;
ros::Publisher pub_yaw_enu_local;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

void registerPub(ros::NodeHandle &n)
{
    //gnss new
    pub_gnss_lla = n.advertise<sensor_msgs::NavSatFix>("gnss_fused_lla", 1000);
    pub_enu_path = n.advertise<nav_msgs::Path>("gnss_enu_path", 1000);
    pub_anc_lla = n.advertise<sensor_msgs::NavSatFix>("gnss_anchor_lla", 1000);
    pub_enu_pose = n.advertise<geometry_msgs::PoseStamped>("enu_pose", 1000);

    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_latest_pure_odometry = n.advertise<nav_msgs::Odometry>("pure_imu_propagate", 1000);
    pub_latest_wheel_odometry = n.advertise<nav_msgs::Odometry>("wheel_propagate", 1000);
    pub_latest_pure_wheel_odometry = n.advertise<nav_msgs::Odometry>("pure_wheel_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    // pub_groundtruth = n.advertise<nav_msgs::Path>("groundtruth", 1000);
    pub_wheel_preintegration = n.advertise<nav_msgs::Path>("wheel_preintegration", 1000);
    pub_pure_wheel_preintegration = n.advertise<nav_msgs::Path>("pure_wheel_preintegration", 1000);
    pub_imu_preintegration = n.advertise<nav_msgs::Path>("imu_preintegration", 1000);
    pub_pure_imu_preintegration = n.advertise<nav_msgs::Path>("pure_imu_preintegration", 1000);
    pub_wheel_rawodom = n.advertise<nav_msgs::Path>("wheel_rawodom", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    // pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);        // new
    pub_lines = n.advertise<visualization_msgs::Marker>("lines_cloud", 1000);              // new
    pub_marg_lines = n.advertise<visualization_msgs::Marker>("history_lines_cloud", 1000); // new
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}
void pubLatestPureOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_pure_odometry.publish(odometry);
}
void pubWheelLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_wheel_odometry.publish(odometry);
}
void pubPureWheelLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_pure_wheel_odometry.publish(odometry);
}
void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}

void printStatistics(const Estimator &estimator, double t)
{
    //    if(ESTIMATE_INTRINSIC_WHEEL) {
    //        std::ofstream ofs(INTRINSIC_ITERATE_PATH, ios::app);
    //        if (!ofs.is_open()) {
    //            ROS_WARN("cannot open %s", INTRINSIC_ITERATE_PATH.c_str());
    //        }
    //        ofs << estimator.sx << " " << estimator.sy << " " << estimator.sw << std::endl;
    //    }
    //    if(ESTIMATE_EXTRINSIC_WHEEL) {
    //        std::ofstream ofs(EXTRINSIC_WHEEL_ITERATE_PATH, ios::app);
    //        if (!ofs.is_open()) {
    //            ROS_WARN("cannot open %s", EXTRINSIC_WHEEL_ITERATE_PATH.c_str());
    //        }
    //        ofs << estimator.tio.transpose() <<" "<< Utility::R2ypr(estimator.rio).transpose()<<std::endl;
    //    }
    //    if(ESTIMATE_EXTRINSIC) {
    //        std::ofstream ofs(EXTRINSIC_CAM_ITERATE_PATH, ios::app);
    //        if (!ofs.is_open()) {
    //            ROS_WARN("cannot open %s", EXTRINSIC_CAM_ITERATE_PATH.c_str());
    //        }
    //        ofs << estimator.tic[0].transpose() <<" "<< Utility::R2ypr(estimator.ric[0]).transpose()<<std::endl;
    //    }

    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    // printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC || ESTIMATE_EXTRINSIC_WHEEL || USE_PLANE)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        if (ESTIMATE_EXTRINSIC)
        {
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                // ROS_DEBUG("calibration result for camera %d", i);
                ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
                ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

                Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
                eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
                eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
                cv::Mat cv_T;
                cv::eigen2cv(eigen_T, cv_T);
                if (i == 0)
                    fs << "body_T_cam0" << cv_T;
                else
                    fs << "body_T_cam1" << cv_T;
            }
        }

        if (ESTIMATE_EXTRINSIC_WHEEL)
        {
            // ROS_DEBUG("calibration result for camera %d", i);
            ROS_DEBUG_STREAM("extirnsic tio: " << estimator.tio.transpose());
            ROS_DEBUG_STREAM("extrinsic rio: " << Utility::R2ypr(estimator.rio).transpose());

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.rio;
            eigen_T.block<3, 1>(0, 3) = estimator.tio;
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            fs << "body_T_wheel" << cv_T;
        }

        if (USE_PLANE)
        {
            ROS_DEBUG_STREAM("plane zpw: " << estimator.zpw);
            ROS_DEBUG_STREAM("plane rpw: " << Utility::R2ypr(estimator.rpw).transpose());

            Eigen::Matrix3d eigen_T = estimator.rpw;
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            fs << "plane_R_world" << cv_T;
            fs << "plane_Z_world" << estimator.zpw;
        }

        fs.release();
    }
    if (ESTIMATE_INTRINSIC_WHEEL)
    {
        cv::FileStorage fs(IN_CALIB_RESULT_PATH, cv::FileStorage::WRITE);

        if (ESTIMATE_INTRINSIC_WHEEL)
        {
            // ROS_DEBUG("calibration result for camera %d", i);
            ROS_DEBUG("intirnsic sx: %f,  sy: %f,  sw: %f", estimator.sx, estimator.sy, estimator.sw);
            fs << "sx" << estimator.sx;
            fs << "sy" << estimator.sy;
            fs << "sw" << estimator.sw;
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
    {
        ROS_INFO("td %f", estimator.td);
        std::ofstream ofs(TD_PATH, ios::app);
        if (!ofs.is_open())
        {
            ROS_WARN("cannot open %s", TD_PATH.c_str());
        }
        ofs << estimator.td << std::endl;
    }

    if (ESTIMATE_TD_WHEEL)
    {
        ROS_INFO("td_wheel %f", estimator.td_wheel);
        std::ofstream ofs(TD_WHEEL_PATH, ios::app);
        if (!ofs.is_open())
        {
            ROS_WARN("cannot open %s", TD_WHEEL_PATH.c_str());
        }
        ofs << estimator.td_wheel << std::endl;
    }
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    //cout<<"reach here"<<endl;
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR||(is_imu_excited))
    {

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        //        // write result to file
        //        ofstream foutC(VINS_RESULT_PATH, ios::app);
        //        foutC.setf(ios::fixed, ios::floatfield);
        //        foutC.precision(0);
        //        foutC << header.stamp.toSec() * 1e9 << ",";
        //        foutC.precision(5);
        //        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
        //              << estimator.Ps[WINDOW_SIZE].y() << ","
        //              << estimator.Ps[WINDOW_SIZE].z() << ","
        //              << tmp_Q.w() << ","
        //              << tmp_Q.x() << ","
        //              << tmp_Q.y() << ","
        //              << tmp_Q.z() << ","
        //              << estimator.Vs[WINDOW_SIZE].x() << ","
        //              << estimator.Vs[WINDOW_SIZE].y() << ","
        //              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        //        foutC.close();
        //        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        //        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
        //                                                          tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC << std::setprecision(9)
              << header.stamp.toSec() << " "
              << std::setprecision(9)
              << pose_stamped.pose.position.x << " "
              << pose_stamped.pose.position.y << " "
              << pose_stamped.pose.position.z << " "
              << pose_stamped.pose.orientation.x << " "
              << pose_stamped.pose.orientation.y << " "
              << pose_stamped.pose.orientation.z << " "
              << pose_stamped.pose.orientation.w << std::endl;
        auto tmp_T = pose_stamped.pose.position;
        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x, tmp_T.y, tmp_T.z,
               tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());

               foutC.close();
               pubGnssResult(estimator, header);
    }
}
// void pubGroundTruth(Estimator &estimator, const std_msgs::Header &header, Eigen::Matrix<double, 7, 1>& pose, const double td)
// {
//     estimator.mGTBuf.lock();
//     if(estimator.groundtruthBuf.empty()){
//         estimator.mGTBuf.unlock();
//         return;
//     }
//     double groundtruth_time = estimator.groundtruthBuf.front().first;
//     double header_time = header.stamp.toSec() - OFFSET_SIM;
//     pose = estimator.groundtruthBuf.front().second;
//     while(groundtruth_time < (header_time - 1e-5)){
//         estimator.groundtruthBuf.pop();
//         if(estimator.groundtruthBuf.empty())
//             break;
//         groundtruth_time = estimator.groundtruthBuf.front().first;
//         pose = estimator.groundtruthBuf.front().second;
//     }
//     if(!estimator.groundtruthBuf.empty() && groundtruth_time > (header_time + 1e-5))
//     {

//         ROS_INFO("wait for new frame");
//         ROS_INFO("groundtruth_time: %f, header_time: %f", groundtruth_time, header_time);
//         estimator.mGTBuf.unlock();
//         return;
//     }
//     if(estimator.groundtruthBuf.empty() || abs(groundtruth_time - header_time)>1e-5)
//     {
//         ROS_ERROR("can not find corresponding groundtruth");
//     }
//     estimator.mGTBuf.unlock();

//     if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
//     {
//         geometry_msgs::PoseStamped pose_stamped;
//         pose_stamped.header = header;
//         pose_stamped.header.frame_id = "world";
//         pose_stamped.pose.orientation.w = pose[0];
//         pose_stamped.pose.orientation.x = pose[1];
//         pose_stamped.pose.orientation.y = pose[2];
//         pose_stamped.pose.orientation.z = pose[3];
//         pose_stamped.pose.position.x = pose[4];
//         pose_stamped.pose.position.y = pose[5];
//         pose_stamped.pose.position.z = pose[6];
//         groundtruth_path.header = header;
//         groundtruth_path.header.frame_id = "world";
//         groundtruth_path.poses.push_back(pose_stamped);
//         pub_groundtruth.publish(groundtruth_path);

//         // write result to file
//         ofstream foutC(GROUNDTRUTH_PATH, ios::app);
//         foutC.setf(ios::fixed, ios::floatfield);
//         foutC << std::setprecision(0)
//         << header.stamp.toSec()<< " "
//         << std::setprecision(9)
//         << pose_stamped.pose.position.x << " "
//         << pose_stamped.pose.position.y << " "
//         << pose_stamped.pose.position.z << " "
//         << pose_stamped.pose.orientation.x << " "
//         << pose_stamped.pose.orientation.y << " "
//         << pose_stamped.pose.orientation.z << " "
//         << pose_stamped.pose.orientation.w << std::endl;

// //        foutC.setf(ios::fixed, ios::floatfield);
// //        foutC.precision(0);
// //        foutC << header.stamp.toSec() * 1e9 << ",";
// //        foutC.precision(5);
// //        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
// //              << estimator.Ps[WINDOW_SIZE].y() << ","
// //              << estimator.Ps[WINDOW_SIZE].z() << ","
// //              << tmp_Q.w() << ","
// //              << tmp_Q.x() << ","
// //              << tmp_Q.y() << ","
// //              << tmp_Q.z() << ","
// //              << estimator.Vs[WINDOW_SIZE].x() << ","
// //              << estimator.Vs[WINDOW_SIZE].y() << ","
// //              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
// //        foutC.close();
// //        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
// //        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
// //               tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());

//         auto tmp_T = pose_stamped.pose.position;
//         auto tmp_Q = pose_stamped.pose.orientation;
//         printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x, tmp_T.y, tmp_T.z,
//                tmp_Q.w, tmp_Q.x, tmp_Q.y, tmp_Q.z);

//     }
// }
void pubGnssResult(const Estimator &estimator, const std_msgs::Header &header)
{
    if (!estimator.gnss_ready)      return;
    // publish GNSS LLA
    const double gnss_ts = estimator.Headers[WINDOW_SIZE] + 
        estimator.diff_t_gnss_local;
    Eigen::Vector3d lla_pos = ecef2geo(estimator.ecef_pos);
    //cout<<"lla pos here"<<estimator.ecef_pos<<endl; //no problem
    //printf("global time: %f\n", gnss_ts);
    //printf("latitude longitude altitude: %f, %f, %f\n", lla_pos.x(), lla_pos.y(), lla_pos.z());
    sensor_msgs::NavSatFix gnss_lla_msg;
    gnss_lla_msg.header.stamp = ros::Time(gnss_ts);
    gnss_lla_msg.header.frame_id = "geodetic";
    gnss_lla_msg.latitude = lla_pos.x();
    gnss_lla_msg.longitude = lla_pos.y();
    gnss_lla_msg.altitude = lla_pos.z();
    pub_gnss_lla.publish(gnss_lla_msg);

    // publish anchor LLA
    const Eigen::Vector3d anc_lla = ecef2geo(estimator.anc_ecef);
    //cout<<"ecef pos here"<<estimator.anc_ecef<<endl;//all zero
    sensor_msgs::NavSatFix anc_lla_msg;
    anc_lla_msg.header = gnss_lla_msg.header;
    anc_lla_msg.latitude = anc_lla.x();
    anc_lla_msg.longitude = anc_lla.y();
    anc_lla_msg.altitude = anc_lla.z();
    pub_anc_lla.publish(anc_lla_msg);

    // publish ENU pose and path
    geometry_msgs::PoseStamped enu_pose_msg;
    // camera-front orientation
    Eigen::Matrix3d R_s_c;
    R_s_c <<  0,  0,  1,
             -1,  0,  0,
              0, -1,  0;
    Eigen::Matrix3d R_w_sensor = estimator.Rs[WINDOW_SIZE] * estimator.ric[0] * R_s_c.transpose();
    Eigen::Quaterniond enu_ori(estimator.R_enu_local * R_w_sensor);
    enu_pose_msg.header.stamp = header.stamp;
    enu_pose_msg.header.frame_id = "world";     // "enu" will more meaningful, but for viz
    enu_pose_msg.pose.position.x = estimator.enu_pos.x();
    enu_pose_msg.pose.position.y = estimator.enu_pos.y();
    enu_pose_msg.pose.position.z = estimator.enu_pos.z();
    enu_pose_msg.pose.orientation.x = enu_ori.x();
    enu_pose_msg.pose.orientation.y = enu_ori.y();
    enu_pose_msg.pose.orientation.z = enu_ori.z();
    enu_pose_msg.pose.orientation.w = enu_ori.w();
    pub_enu_pose.publish(enu_pose_msg);

    enu_path.header = enu_pose_msg.header;
    enu_path.poses.push_back(enu_pose_msg);
    pub_enu_path.publish(enu_path);

    // publish ENU-local tf
    Eigen::Quaterniond q_enu_world(estimator.R_enu_local);
    static tf::TransformBroadcaster br;
    tf::Transform transform_enu_world;

    transform_enu_world.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion tf_q;
    tf_q.setW(q_enu_world.w());
    tf_q.setX(q_enu_world.x());
    tf_q.setY(q_enu_world.y());
    tf_q.setZ(q_enu_world.z());
    transform_enu_world.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform_enu_world, header.stamp, "enu", "world"));

    // write GNSS result to file
    ofstream gnss_output(GNSS_RESULT_PATH, ios::app);

      gnss_output.setf(ios::fixed, ios::floatfield);
    gnss_output.precision(9);
    gnss_output << header.stamp.toSec() << ' ';
    //gnss_output << gnss_ts * 1e9 << ',';
    gnss_output.precision(9);
    gnss_output << estimator.ecef_pos(0) << ' '
                << estimator.ecef_pos(1) << ' '
                << estimator.ecef_pos(2) << ' '
                << 0.000000000 << " "
        << 0.000000000 << " "
        << 0.000000000 << " "
        << 0.000000000 << std::endl;


                // << estimator.yaw_enu_local << ','
                // << estimator.para_rcv_dt[(WINDOW_SIZE)*4+0] << ','
                // << estimator.para_rcv_dt[(WINDOW_SIZE)*4+1] << ','
                // << estimator.para_rcv_dt[(WINDOW_SIZE)*4+2] << ','
                // << estimator.para_rcv_dt[(WINDOW_SIZE)*4+3] << ','
                // << estimator.para_rcv_ddt[WINDOW_SIZE] << ','
                // << estimator.anc_ecef(0) << ','
                // << estimator.anc_ecef(1) << ','
                // << estimator.anc_ecef(2) << '\n';


    // gnss_output.setf(ios::fixed, ios::floatfield);
    // gnss_output.precision(0);
    // gnss_output << header.stamp.toSec() * 1e9 << ',';
    // gnss_output << gnss_ts * 1e9 << ',';
    // gnss_output.precision(5);
    // gnss_output << estimator.ecef_pos(0) << ','
    //             << estimator.ecef_pos(1) << ','
    //             << estimator.ecef_pos(2) << ','
    //             << estimator.yaw_enu_local << ','
    //             << estimator.para_rcv_dt[(WINDOW_SIZE)*4+0] << ','
    //             << estimator.para_rcv_dt[(WINDOW_SIZE)*4+1] << ','
    //             << estimator.para_rcv_dt[(WINDOW_SIZE)*4+2] << ','
    //             << estimator.para_rcv_dt[(WINDOW_SIZE)*4+3] << ','
    //             << estimator.para_rcv_ddt[WINDOW_SIZE] << ','
    //             << estimator.anc_ecef(0) << ','
    //             << estimator.anc_ecef(1) << ','
    //             << estimator.anc_ecef(2) << '\n';
    gnss_output.close();
}

void pubIMUPreintegration(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::Header &header)
{
    static nav_msgs::Path preintegration_path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = Q.w();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.position.x = P[0];
    pose_stamped.pose.position.y = P[1];
    pose_stamped.pose.position.z = P[2];
    preintegration_path.header = header;
    preintegration_path.header.frame_id = "world";
    preintegration_path.poses.push_back(pose_stamped);
    pub_imu_preintegration.publish(preintegration_path);
}

void pubPureIMUPreintegration(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::Header &header)
{
    static nav_msgs::Path preintegration_path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = Q.w();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.position.x = P[0];
    pose_stamped.pose.position.y = P[1];
    pose_stamped.pose.position.z = P[2];
    preintegration_path.header = header;
    preintegration_path.header.frame_id = "world";
    preintegration_path.poses.push_back(pose_stamped);
    pub_pure_imu_preintegration.publish(preintegration_path);
}

void pubWheelPreintegration(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::Header &header)
{
    static nav_msgs::Path preintegration_path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = Q.w();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.position.x = P[0];
    pose_stamped.pose.position.y = P[1];
    pose_stamped.pose.position.z = P[2];
    preintegration_path.header = header;
    preintegration_path.header.frame_id = "world";
    preintegration_path.poses.push_back(pose_stamped);
    pub_wheel_preintegration.publish(preintegration_path);
}


void pubPureWheelPreintegration(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::Header &header)
{
    static nav_msgs::Path preintegration_path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = Q.w();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.position.x = P[0];
    pose_stamped.pose.position.y = P[1];
    pose_stamped.pose.position.z = P[2];
    preintegration_path.header = header;
    preintegration_path.header.frame_id = "world";
    preintegration_path.poses.push_back(pose_stamped);
    pub_pure_wheel_preintegration.publish(preintegration_path);
}

void pubWheelRawodom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::Header &header)
{
    static nav_msgs::Path rawodom_path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = Q.w();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.position.x = P[0];
    pose_stamped.pose.position.y = P[1];
    pose_stamped.pose.position.z = P[2];
    rawodom_path.header = header;
    rawodom_path.header.frame_id = "world";
    rawodom_path.poses.push_back(pose_stamped);
    pub_wheel_rawodom.publish(rawodom_path);
    // txt output
    ofstream foutC(VINS_RESULT_PATH2, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC << std::setprecision(9)
          << header.stamp.toSec() << " "
          << std::setprecision(9)
          << pose_stamped.pose.position.x << " "
          << pose_stamped.pose.position.y << " "
          << pose_stamped.pose.position.z << " "
          << pose_stamped.pose.orientation.x << " "
          << pose_stamped.pose.orientation.y << " "
          << pose_stamped.pose.orientation.z << " "
          << pose_stamped.pose.orientation.w << std::endl;
    auto tmp_T = pose_stamped.pose.position;
    // printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x, tmp_T.y, tmp_T.z,
    //        tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
}
void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    // static int key_poses_id = 0;
    key_poses.id = 0; // key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        if (STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

visualization_msgs::Marker marg_lines_cloud; // 全局变量用来保存所有的线段
std::list<visualization_msgs::Marker> marg_lines_cloud_last10frame;
void pubLinesCloud(const Estimator &estimator, const std_msgs::Header &header) // new
{
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.header.frame_id = "world";
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.lifetime = ros::Duration();

    // static int key_poses_id = 0;
    lines.id = 0; // key_poses_id++;
    lines.scale.x = 0.03;
    lines.scale.y = 0.03;
    lines.scale.z = 0.03;
    lines.color.b = 1.0;
    lines.color.a = 1.0;

    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
        //        int used_num;
        //        used_num = it_per_id.linefeature_per_frame.size();
        //
        //        if (!(used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))
        //            continue;
        //        //if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.is_triangulation == false)
            continue;

        // std::cout<< "used num: " <<used_num<<" line id: "<<it_per_id.feature_id<<std::endl;

        int imu_i = it_per_id.start_frame;

        Vector3d pc, nc, vc;
        // pc = it_per_id.line_plucker.head(3);
        // nc = pc.cross(vc);
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);
        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs; // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
        Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
        Vector2d ln = (p11.cross(p21)).head(2); // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0); // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d(0, 0, 0);

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        //
        //        if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
        //            continue;

        // std::cout <<"visual: "<< it_per_id.feature_id <<" " << it_per_id.line_plucker <<"\n\n";
        Vector3d pts_1(e1(0), e1(1), e1(2));
        Vector3d pts_2(e2(0), e2(1), e2(2));

        Vector3d w_pts_1 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0]) + estimator.Ps[imu_i];
        Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0]) + estimator.Ps[imu_i];

        /*
                Vector3d diff_1 = it_per_id.ptw1 - w_pts_1;
                Vector3d diff_2 = it_per_id.ptw2 - w_pts_2;
                if(diff_1.norm() > 1 || diff_2.norm() > 1)
                {
                    std::cout <<"visual: "<<it_per_id.removed_cnt<<" "<<it_per_id.all_obs_cnt<<" " << it_per_id.feature_id <<"\n";// << it_per_id.line_plucker <<"\n\n" << it_per_id.line_plk_init <<"\n\n";
                    std::cout << it_per_id.Rj_ <<"\n" << it_per_id.tj_ <<"\n\n";
                    std::cout << estimator.Rs[imu_i] <<"\n" << estimator.Ps[imu_i] <<"\n\n";
                    std::cout << obs <<"\n\n" << it_per_id.obs_j<<"\n\n";

                }

                w_pts_1 = it_per_id.ptw1;
                w_pts_2 = it_per_id.ptw2;
        */
        /*
                Vector3d w_pts_1 =  estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                                   + estimator.Ps[imu_i];
                Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                                   + estimator.Ps[imu_i];

                Vector3d d = w_pts_1 - w_pts_2;
                if(d.norm() > 4.0 || d.norm() < 2.0)
                    continue;
        */
        geometry_msgs::Point p;
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        lines.points.push_back(p);
        p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        lines.points.push_back(p);
    }
    // std::cout<<" viewer lines.size: " <<lines.points.size() << std::endl;
    pub_lines.publish(lines);

    //    visualization_msgs::Marker marg_lines_cloud_oneframe; // 最近一段时间的
    //    marg_lines_cloud_oneframe.header = header;
    //    marg_lines_cloud_oneframe.header.frame_id = "world";
    //    marg_lines_cloud_oneframe.ns = "lines";
    //    marg_lines_cloud_oneframe.type = visualization_msgs::Marker::LINE_LIST;
    //    marg_lines_cloud_oneframe.action = visualization_msgs::Marker::ADD;
    //    marg_lines_cloud_oneframe.pose.orientation.w = 1.0;
    //    marg_lines_cloud_oneframe.lifetime = ros::Duration();
    //
    //    //marg_lines_cloud.id = 0; //key_poses_id++;
    //    marg_lines_cloud_oneframe.scale.x = 0.05;
    //    marg_lines_cloud_oneframe.scale.y = 0.05;
    //    marg_lines_cloud_oneframe.scale.z = 0.05;
    //    marg_lines_cloud_oneframe.color.g = 1.0;
    //    marg_lines_cloud_oneframe.color.a = 1.0;

    //////////////////////////////////////////////
    // all marglization line
    marg_lines_cloud.header = header;
    marg_lines_cloud.header.frame_id = "world";
    marg_lines_cloud.ns = "lines";
    marg_lines_cloud.type = visualization_msgs::Marker::LINE_LIST;
    marg_lines_cloud.action = visualization_msgs::Marker::ADD;
    marg_lines_cloud.pose.orientation.w = 1.0;
    marg_lines_cloud.lifetime = ros::Duration();

    // static int key_poses_id = 0;
    // marg_lines_cloud.id = 0; //key_poses_id++;
    marg_lines_cloud.scale.x = 0.05;
    marg_lines_cloud.scale.y = 0.05;
    marg_lines_cloud.scale.z = 0.05;
    marg_lines_cloud.color.r = 1.0;
    marg_lines_cloud.color.a = 1.0;
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
        //        int used_num;
        //        used_num = it_per_id.linefeature_per_frame.size();
        //        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        //            continue;
        //        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.linefeature_per_frame.size() <= 2 && it_per_id.is_triangulation == true)
        {
            int imu_i = it_per_id.start_frame;

            Vector3d pc, nc, vc;
            // pc = it_per_id.line_plucker.head(3);
            // nc = pc.cross(vc);
            nc = it_per_id.line_plucker.head(3);
            vc = it_per_id.line_plucker.tail(3);
            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs; // 第一次观测到这帧
            Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
            Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
            Vector2d ln = (p11.cross(p21)).head(2); // 直线的垂直方向
            ln = ln / ln.norm();

            Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0); // 直线垂直方向上移动一个单位
            Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            Vector3d cam = Vector3d(0, 0, 0);

            Vector4d pi1 = pi_from_ppp(cam, p11, p12);
            Vector4d pi2 = pi_from_ppp(cam, p21, p22);

            Vector4d e1 = Lc * pi1;
            Vector4d e2 = Lc * pi2;
            e1 = e1 / e1(3);
            e2 = e2 / e2(3);

            //            if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
            //                continue;
            //
            double length = (e1 - e2).norm();
            if (length > 10)
                continue;

            // std::cout << e1 <<"\n\n";
            Vector3d pts_1(e1(0), e1(1), e1(2));
            Vector3d pts_2(e2(0), e2(1), e2(2));

            Vector3d w_pts_1 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0]) + estimator.Ps[imu_i];
            Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0]) + estimator.Ps[imu_i];

            // w_pts_1 = it_per_id.ptw1;
            // w_pts_2 = it_per_id.ptw2;

            geometry_msgs::Point p;
            p.x = w_pts_1(0);
            p.y = w_pts_1(1);
            p.z = w_pts_1(2);
            marg_lines_cloud.points.push_back(p);
            //            marg_lines_cloud_oneframe.points.push_back(p);
            p.x = w_pts_2(0);
            p.y = w_pts_2(1);
            p.z = w_pts_2(2);
            marg_lines_cloud.points.push_back(p);
            //            marg_lines_cloud_oneframe.points.push_back(p);
        }
    }
    //    if(marg_lines_cloud_oneframe.points.size() > 0)
    //        marg_lines_cloud_last10frame.push_back(marg_lines_cloud_oneframe);
    //
    //    if(marg_lines_cloud_last10frame.size() > 50)
    //        marg_lines_cloud_last10frame.pop_front();
    //
    //    marg_lines_cloud.points.clear();
    //    list<visualization_msgs::Marker>::iterator itor;
    //    itor = marg_lines_cloud_last10frame.begin();
    //    while(itor != marg_lines_cloud_last10frame.end())
    //    {
    //        for (int i = 0; i < itor->points.size(); ++i) {
    //            marg_lines_cloud.points.push_back(itor->points.at(i));
    //        }
    //        itor++;
    //    }

    //    ofstream foutC("/home/hyj/catkin_ws/src/VINS-Mono/config/euroc/landmark.txt");
    //    for (int i = 0; i < marg_lines_cloud.points.size();) {
    //
    //        geometry_msgs::Point pt1 = marg_lines_cloud.points.at(i);
    //        geometry_msgs::Point pt2 = marg_lines_cloud.points.at(i+1);
    //        i = i + 2;
    //        foutC << pt1.x << " "
    //              << pt1.y << " "
    //              << pt1.z << " "
    //              << pt2.x << " "
    //              << pt2.y << " "
    //              << pt2.z << "\n";
    //    }
    //    foutC.close();
    pub_marg_lines.publish(marg_lines_cloud);
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    ROS_INFO("good point size: %d", point_cloud.points.size());
    double curTime = header.stamp.toSec();
   
    pub_point_cloud.publish(point_cloud);

    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        // if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //         continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1)
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    // wheel frame
    if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL&&wheel_ready)
    {
        transform.setOrigin(tf::Vector3(estimator.tio.x(),
                                        estimator.tio.y(),
                                        estimator.tio.z()));
        q.setW(Quaterniond(estimator.rio).w());
        q.setX(Quaterniond(estimator.rio).x());
        q.setY(Quaterniond(estimator.rio).y());
        q.setZ(Quaterniond(estimator.rio).z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "wheel"));
    }

    // plane frame
    if (USE_PLANE)
    {
        transform.setOrigin(tf::Vector3(0,
                                        0,
                                        estimator.zpw));
        q.setW(Quaterniond(estimator.rpw).w());
        q.setX(Quaterniond(estimator.rpw).x());
        q.setY(Quaterniond(estimator.rpw).y());
        q.setZ(Quaterniond(estimator.rpw).z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, header.stamp, "plane", "world"));
        //        std::cout<<"plane rpw: "<< Eigen::Quaterniond(estimator.rpw).coeffs().transpose()<<" zpw: "<<estimator.zpw<<std::endl;
        ROS_DEBUG_STREAM("plane rpw: " << Eigen::Quaterniond(estimator.rpw).coeffs().transpose() << " zpw: " << estimator.zpw);
    }

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        // Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        // printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);

        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        pub_keyframe_point.publish(point_cloud);
    }
}