/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"
using namespace std;
queue<nav_msgs::Odometry::ConstPtr> wheel_odom_buf;
std::mutex m_wheel_odom;
int lineFeaturePerId::endFrame() // line
{
    return start_frame + linefeature_per_frame.size() - 1;
}
int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if (it->feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    // if (frame_count < 2 || last_track_num < 20)
    // if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::addFeatureCheckParallaxwithline(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines, double td)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount()); // 已有的特征数目
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image) // 遍历当前帧上的特征
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td); // add td

        int feature_id = id_pts.first;
        // std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
                              return it.feature_id == feature_id; // 在feature里找id号为feature_id的特征
                          });

        if (it == feature.end()) // 如果之前没存这个特征，说明是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++; // 已经跟踪上了多少个点
        }
    }

    for (auto &id_line : lines) // 遍历当前帧上的特征
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second); // 观测

        int feature_id = id_line.first;
        // cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it) // linefeature can not be found
                          {
                              return it.feature_id == feature_id; // 在feature里找id号为feature_id的特征
                          });

        if (it == linefeature.end()) // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

// original
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            double depth_a = it.feature_per_frame[idx_l].depth;
            double depth_b = it.feature_per_frame[idx_r].depth;
            // cout<<"depth a b:"<<depth_a<<"  "<<depth_b<<endl;
            if (depth_a < 0.1 || depth_a > 10) // max and min measurement
                continue;

            if (depth_b < 0.1 || depth_b > 10) // max and min measurement
                continue;
            a = it.feature_per_frame[idx_l].point;
            b = it.feature_per_frame[idx_r].point;
            a = a * depth_a;
            b = b * depth_b;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                      Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,
                                    vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    // printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    // pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if (!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    // cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if (frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if ((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d);
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if (solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose();
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            // cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            // cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

void FeatureManager::linear_insert(Eigen::Quaterniond &Qodom, Eigen::Vector3d &Podom, const double sync_time, nav_msgs::Odometry::ConstPtr &front_data, nav_msgs::Odometry::ConstPtr &back_data)
{

    double front_scale = (back_data->header.stamp.toSec() - sync_time) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    double back_scale = (sync_time - front_data->header.stamp.toSec()) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    // ���Բ�ֵλ��
    // ֻ��ֵx,y����ת��z,w
    Podom << front_data->pose.pose.position.x * front_scale + back_data->pose.pose.position.x * back_scale,
        front_data->pose.pose.position.y * front_scale + back_data->pose.pose.position.y * back_scale,
        // front_data->pose.pose.position.z* front_scale + back_data->pose.pose.position.z * back_scale;
        0;
    // ���Բ�ֵ��ת
    // Qodom.x() = front_data->pose.pose.orientation.x*front_scale+back_data->pose.pose.orientation.x*back_scale;
    // Qodom.y() = front_data->pose.pose.orientation.y * front_scale + back_data->pose.pose.orientation.y * back_scale;
    Qodom.x() = 0.0;
    Qodom.y() = 0.0;
    Qodom.z() = front_data->pose.pose.orientation.z * front_scale + back_data->pose.pose.orientation.z * back_scale;
    Qodom.w() = front_data->pose.pose.orientation.w * front_scale + back_data->pose.pose.orientation.w * back_scale;
    Qodom.normalize();
}

void FeatureManager::getOdomData(Eigen::Quaterniond &Q, Eigen::Vector3d &P, nav_msgs::Odometry::ConstPtr &odomData)
{
    // ȡ��ת

    Q.x() = -odomData->pose.pose.orientation.x;
    Q.y() = -odomData->pose.pose.orientation.y;
    Q.z() = odomData->pose.pose.orientation.z;
    Q.w() = odomData->pose.pose.orientation.w;

    // ȡλ��
    P << odomData->pose.pose.position.x, odomData->pose.pose.position.y, odomData->pose.pose.position.z;
}

bool FeatureManager::getPoseByWheelOdom(Eigen::Vector3d &Pcam, Eigen::Matrix3d &Rcam, const double curTime)
{

    if (wheel_odom_buf.empty())
    {
        printf("odom data has not push into buf yet! \n");
        return false;
    }

    nav_msgs::Odometry::ConstPtr odomFront;
    nav_msgs::Odometry::ConstPtr odomBack;
    Eigen::Quaterniond odomQ;
    Eigen::Vector3d odomP;

    // wheel��cam��ת����

    Eigen::Matrix3d wheel2Cam;
    Eigen::Matrix<double, 1, 3> Ttemp;
    wheel2Cam << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    if (curTime < wheel_odom_buf.front()->header.stamp.toSec() || wheel_odom_buf.back()->header.stamp.toSec() < curTime)
    {
        printf("curtime odom data not push into wheel buf! \n ");
        return false;
    }
    else
    {
        // ��ʱ��
        TicToc use_time;

        m_wheel_odom.lock();
        while (wheel_odom_buf.front()->header.stamp.toSec() <= curTime)
        {
            if (wheel_odom_buf.front()->header.stamp.toSec() == curTime)
            {
                getOdomData(odomQ, odomP, wheel_odom_buf.front());
                // printf("get odom curtime data: q %f %f %f %f \n",odomQ.x(), odomQ.y(), odomQ.z(), odomQ.w());
                cout << odomP << endl;
                // �������ϵ��world�µ�λ��
                odomQ.normalize();
                Rcam = odomQ.toRotationMatrix();
                // Ttemp=Twc.transpose()* Rcam.transpose();
                Pcam = odomP + Rcam * TIO;
                // �������ϵ����ת
                Rcam *= wheel2Cam; // 7-8
                // ��base_linkת����camera

                // transformOdomTCam(Rcam,Pcam);

                // Rcam = odomQ.toRotationMatrix();
                wheel_odom_buf.pop();
                m_wheel_odom.unlock();
                return true;
            }
            odomFront = wheel_odom_buf.front();
            wheel_odom_buf.pop();
        }
        odomBack = wheel_odom_buf.front();
        m_wheel_odom.unlock();
        linear_insert(odomQ, odomP, curTime, odomFront, odomBack);
        // printf("get odom curtime data: q %f %f %f %f \n", odomQ.x(), odomQ.y(), odomQ.z(), odomQ.w());
        // cout << odomP << endl;
        // �������ϵ��world�µ�λ��
        Rcam = odomQ.toRotationMatrix();
        // Ttemp = Twc.transpose() * Rcam;
        // Ttemp = Twc.transpose() * Rcam.transpose();
        Pcam = odomP + Rcam * TIO;
        // Pcam = odomP + Rcam * Twc;
        // �������ϵ����ת
        Rcam *= wheel2Cam; // 7-8

        // Rcam = odomQ.toRotationMatrix();
        // ��ʱ
        // printf("get pose data from wheel odom cost %f \n",use_time.toc());
        return true;
    }
}

void FeatureManager::initFramePoseByOdom(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], const double curtime)
{

    // printf("enter initFramePoseByPnP! \n");
    if (frameCnt > 0)
    {

        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;

        if (getPoseByWheelOdom(PCam, RCam, curtime))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose();
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            // cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            // cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
        else
        {
            printf("solvePoseByWheel failed! \n");
        }
    }
}

// void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
// {
//     for (auto &it_per_id : feature)
//     {
//         if (it_per_id.estimated_depth > 0)//both have this..   skip if already do it
//             continue;

//         if((STEREO || DEPTH) && it_per_id.feature_per_frame[0].is_stereo) {//cout<<"test if enter stereo"<<endl;
//         {
//             int imu_i = it_per_id.start_frame;
//             Eigen::Matrix<double, 3, 4> leftPose;
//             Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
//             Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
//             leftPose.leftCols<3>() = R0.transpose();
//             leftPose.rightCols<1>() = -R0.transpose() * t0;
//             //cout << "left pose " << leftPose << endl;

//             Eigen::Matrix<double, 3, 4> rightPose;
//             Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
//             Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
//             rightPose.leftCols<3>() = R1.transpose();
//             rightPose.rightCols<1>() = -R1.transpose() * t1;
//             //cout << "right pose " << rightPose << endl;

//             Eigen::Vector2d point0, point1;
//             Eigen::Vector3d point3d;
//             point0 = it_per_id.feature_per_frame[0].point.head(2);
//             point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
//             //cout << "point0 " << point0.transpose() << endl;
//             //cout << "point1 " << point1.transpose() << endl;

//             triangulatePoint(leftPose, rightPose, point0, point1, point3d);
//             Eigen::Vector3d localPoint;
//             localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
//             double depth = localPoint.z();

//             if (depth > 0 && depth < 7) {
//                 it_per_id.estimated_depth = depth;
//             } else
//                 it_per_id.estimated_depth = INIT_DEPTH;//5
//             continue;
//         }//before change

//         }// for rgbd, only look up
//         else if(it_per_id.feature_per_frame.size() > 1)
//         {
//             int imu_i = it_per_id.start_frame;
//             Eigen::Matrix<double, 3, 4> leftPose;
//             Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
//             Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
//             leftPose.leftCols<3>() = R0.transpose();
//             leftPose.rightCols<1>() = -R0.transpose() * t0;

//             imu_i++;
//             Eigen::Matrix<double, 3, 4> rightPose;
//             Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
//             Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
//             rightPose.leftCols<3>() = R1.transpose();
//             rightPose.rightCols<1>() = -R1.transpose() * t1;

//             Eigen::Vector2d point0, point1;
//             Eigen::Vector3d point3d;
//             point0 = it_per_id.feature_per_frame[0].point.head(2);
//             point1 = it_per_id.feature_per_frame[1].point.head(2);
//             triangulatePoint(leftPose, rightPose, point0, point1, point3d);
//             Eigen::Vector3d localPoint;
//             localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
//             double depth = localPoint.z();
//             if (depth > 0)
//                 it_per_id.estimated_depth = depth;
//             else
//                 it_per_id.estimated_depth = INIT_DEPTH;
//             continue;
//         }
//         //cout<<"test if enter here"<<endl;     below for vinsmono
//         it_per_id.used_num = it_per_id.feature_per_frame.size();
//         if (it_per_id.used_num < 4)
//             continue;

//         int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

//         Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
//         int svd_idx = 0;

//         Eigen::Matrix<double, 3, 4> P0;
//         Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
//         Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
//         P0.leftCols<3>() = Eigen::Matrix3d::Identity();
//         P0.rightCols<1>() = Eigen::Vector3d::Zero();

//         for (auto &it_per_frame : it_per_id.feature_per_frame)
//         {
//             imu_j++;

//             Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
//             Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
//             Eigen::Vector3d t = R0.transpose() * (t1 - t0);
//             Eigen::Matrix3d R = R0.transpose() * R1;
//             Eigen::Matrix<double, 3, 4> P;
//             P.leftCols<3>() = R.transpose();
//             P.rightCols<1>() = -R.transpose() * t;
//             Eigen::Vector3d f = it_per_frame.point.normalized();
//             svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
//             svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

//             if (imu_i == imu_j)
//                 continue;
//         }
//         ROS_ASSERT(svd_idx == svd_A.rows());
//         Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
//         double svd_method = svd_V[2] / svd_V[3];

//         it_per_id.estimated_depth = svd_method;
//         //it_per_id->estimated_depth = INIT_DEPTH;

//         if (it_per_id.estimated_depth < 0.1)
//             it_per_id.estimated_depth = INIT_DEPTH;

//     }

// }

// my own version   do not use depth or stereo
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0) // both have this..   skip if already do it
            continue;

        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        it_per_id.estimate_flag = 2; // triangulate
        // it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
            it_per_id.estimate_flag = 0;
        }
    }
}

// copy from rgbd
void FeatureManager::triangulateWithDepth(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        //     continue;
        // it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) // here can adjust
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;

        int start_frame = it_per_id.start_frame;

        vector<double> verified_depths;

        Eigen::Vector3d tr = Ps[start_frame] + Rs[start_frame] * tic[0];
        Eigen::Matrix3d Rr = Rs[start_frame] * ric[0];

        for (int i = 0; i < (int)it_per_id.feature_per_frame.size(); i++)
        {
            Eigen::Vector3d t0 = Ps[start_frame + i] + Rs[start_frame + i] * tic[0];
            Eigen::Matrix3d R0 = Rs[start_frame + i] * ric[0]; // here should be 10
            // double depth_threshold = 3; //for handheld and wheeled application. Since d435i <3 is quiet acc
            // double depth_threshold = 10; //for tracked application, since IMU quite noisy in this scene
            if (it_per_id.feature_per_frame[i].depth < 0.1 || it_per_id.feature_per_frame[i].depth > depth_threshold)
                continue;
            Eigen::Vector3d point0(it_per_id.feature_per_frame[i].point * it_per_id.feature_per_frame[i].depth);

            // transform to reference frame
            Eigen::Vector3d t2r = Rr.transpose() * (t0 - tr);
            Eigen::Matrix3d R2r = Rr.transpose() * R0;

            for (int j = 0; j < (int)it_per_id.feature_per_frame.size(); j++)
            {
                if (i == j)
                    continue;
                Eigen::Vector3d t1 = Ps[start_frame + j] + Rs[start_frame + j] * tic[0];
                Eigen::Matrix3d R1 = Rs[start_frame + j] * ric[0];
                Eigen::Vector3d t20 = R0.transpose() * (t1 - t0);
                Eigen::Matrix3d R20 = R0.transpose() * R1;

                Eigen::Vector3d point1_projected = R20.transpose() * point0 - R20.transpose() * t20;
                Eigen::Vector2d point1_2d(it_per_id.feature_per_frame[j].point.x(), it_per_id.feature_per_frame[j].point.y());
                Eigen::Vector2d residual = point1_2d - Vector2d(point1_projected.x() / point1_projected.z(), point1_projected.y() / point1_projected.z());
                if (residual.norm() < 10.0 / 460)
                { // this can also be adjust to improve performance
                    Eigen::Vector3d point_r = R2r * point0 + t2r;
                    verified_depths.push_back(point_r.z());
                }
            }
        }
        // cout<<"verified points num:"<<verified_depths.size()<<endl;

        if (verified_depths.size() == 0)
            continue;
        double depth_sum = std::accumulate(std::begin(verified_depths), std::end(verified_depths), 0.0);
        double depth_ave = depth_sum / verified_depths.size();
        //        for (int i=0;i<(int)verified_depths.size();i++){
        //            cout << verified_depths[i]<<"|";
        //        }
        //        cout << endl;
        it_per_id.estimated_depth = depth_ave;
        it_per_id.estimate_flag = 1; // 1

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
            it_per_id.estimate_flag = 0;
        }
    }
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if (itSet != outlierIndex.end())
        {
            feature.erase(it);
            // printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeBackline()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
        // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin()); // 删掉特征ft在这个图像帧上的观测量
            if (it->feature_per_frame.size() == 0)                      // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                feature.erase(it);
        }
    }

    // std::cout << "remove back" << std::endl;
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
        // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin()); // 删掉特征ft在这个图像帧上的观测量
            if (it->linefeature_per_frame.size() == 0)                          // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFrontline(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count) // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame; // j指向第i-1帧
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j); // 删掉特征ft在这个图像帧上的观测量
            if (it->feature_per_frame.size() == 0)                          // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                feature.erase(it);
        }
    }

    // std::cout << "remove front \n";
    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count) // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame; // j指向第i-1帧
            if (it->endFrame() < frame_count - 1)
                continue;
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin() + j); // 删掉特征ft在这个图像帧上的观测量
            if (it->linefeature_per_frame.size() == 0)                              // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

int FeatureManager::getLineFeatureCount()
{
    int cnt = 0;
    for (auto &it : linefeature)
    {

        it.used_num = it.linefeature_per_frame.size();

        if (it.used_num >= LINE_MIN_OBS && it.start_frame < WINDOW_SIZE - 2 && it.is_triangulation)
        {
            cnt++;
        }
    }
    return cnt;
}

MatrixXd FeatureManager::getLineOrthVectorInCamera()
{
    MatrixXd lineorth_vec(getLineFeatureCount(), 4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);
    }
    return lineorth_vec;
}

void FeatureManager::setLineOrthInCamera(MatrixXd x)
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        // std::cout<<"x:"<<x.rows() <<" "<<feature_index<<"\n";
        Vector4d line_orth = x.row(++feature_index);
        it_per_id.line_plucker = orth_to_plk(line_orth); // transfrom to camera frame

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}
MatrixXd FeatureManager::getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    MatrixXd lineorth_vec(getLineFeatureCount(), 4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        int imu_i = it_per_id.start_frame;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric

        Vector6d line_w = plk_to_pose(it_per_id.line_plucker, Rwc, twc); // transfrom to world frame
        // line_w.normalize();
        lineorth_vec.row(++feature_index) = plk_to_orth(line_w);
        // lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);
    }
    return lineorth_vec;
}

void FeatureManager::setLineOrth(MatrixXd x, Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        Vector4d line_orth_w = x.row(++feature_index);
        Vector6d line_w = orth_to_plk(line_orth_w);

        int imu_i = it_per_id.start_frame;
        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = P[imu_i] + R[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = R[imu_i] * ric[0];            // Rwc = Rwi * Ric

        it_per_id.line_plucker = plk_from_pose(line_w, Rwc, twc); // transfrom to camera frame
        // it_per_id.line_plucker = line_w; // transfrom to camera frame

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}

double FeatureManager::reprojection_error(Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w)
{

    double error = 0;

    Vector3d n_w, d_w;
    n_w = line_w.head(3);
    d_w = line_w.tail(3);

    Vector3d p1, p2;
    p1 << obs[0], obs[1], 1;
    p2 << obs[2], obs[3], 1;

    Vector6d line_c = plk_from_pose(line_w, Rwc, twc);
    Vector3d nc = line_c.head(3);
    double sql = nc.head(2).norm();
    nc /= sql;

    error += fabs(nc.dot(p1));
    error += fabs(nc.dot(p2));

    return error / 2.0;
}
//
// void FeatureManager::triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
void FeatureManager::triangulateLine(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    // std::cout<<"linefeature size: "<<linefeature.size()<<std::endl;
    for (auto &it_per_id : linefeature) // 遍历每个特征，对新特征进行三角化
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size(); // 已经有多少帧看到了这个特征
        // cout<<"used num:"<<it_per_id.used_num<<endl;
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2)) // 看到的帧数少于2， 或者 这个特征最近倒数第二帧才看到， 那都不三角化
            continue;

        if (it_per_id.is_triangulation) // 如果已经三角化了
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        // cout<<"num of cam:"<<NUM_OF_CAM<<endl;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric

        double d = 0, min_cos_theta = 1.0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi, obsj; // obs from two frame are used to do triangulation

        // plane pi from ith obs in ith camera frame
        Eigen::Vector4d pii;
        Eigen::Vector3d ni;                                        // normal vector of plane
        for (auto &it_per_frame : it_per_id.linefeature_per_frame) // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j++;

            if (imu_j == imu_i) // 第一个观测是start frame 上
            {
                obsi = it_per_frame.lineobs;
                Eigen::Vector3d p1(obsi(0), obsi(1), 1);
                Eigen::Vector3d p2(obsi(2), obsi(3), 1);
                pii = pi_from_ppp(p1, p2, Vector3d(0, 0, 0));
                ni = pii.head(3);
                ni.normalize();
                continue;
            }

            // 非start frame(其他帧)上的观测
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            Eigen::Vector3d t = R0.transpose() * (t1 - t0); // tij
            Eigen::Matrix3d R = R0.transpose() * R1;        // Rij

            Eigen::Vector4d obsj_tmp = it_per_frame.lineobs;

            // plane pi from jth obs in ith camera frame
            Vector3d p3(obsj_tmp(0), obsj_tmp(1), 1);
            Vector3d p4(obsj_tmp(2), obsj_tmp(3), 1);
            p3 = R * p3 + t;
            p4 = R * p4 + t;
            Vector4d pij = pi_from_ppp(p3, p4, t);
            Eigen::Vector3d nj = pij.head(3);
            nj.normalize();

            double cos_theta = ni.dot(nj);
            if (cos_theta < min_cos_theta)
            {
                min_cos_theta = cos_theta;
                tij = t;
                Rij = R;
                obsj = obsj_tmp;
                d = t.norm();
            }
            // if( d < t.norm() )  // 选择最远的那俩帧进行三角化
            // {
            //     d = t.norm();
            //     tij = t;
            //     Rij = R;
            //     obsj = it_per_frame.lineobs;      // 特征的图像坐标
            // }
        }

        // if the distance between two frame is lower than 0.1m or the parallax angle is lower than 15deg , do not triangulate.
        // if(d < 0.1 || min_cos_theta > 0.998)
        if (min_cos_theta > 0.998)
            // if( d < 0.2 )
            continue;

        // plane pi from jth obs in ith camera frame
        Vector3d p3(obsj(0), obsj(1), 1);
        Vector3d p4(obsj(2), obsj(3), 1);
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4, tij);

        Vector6d plk = pipi_plk(pii, pij);
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        // Vector3d cp = plucker_origin( n, v );
        // if ( cp(2) < 0 )
        {
            //  cp = - cp;
            //  continue;
        }

        // Vector6d line;
        // line.head(3) = cp;
        // line.tail(3) = v;
        // it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk; // plk in camera frame
        it_per_id.is_triangulation = true;

        //  used to debug
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id.linefeature_per_frame[0].lineobs; // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
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

        Vector3d pts_1(e1(0), e1(1), e1(2));
        Vector3d pts_2(e2(0), e2(1), e2(2));

        Vector3d w_pts_1 = Rs[imu_i] * (ric[0] * pts_1 + tic[0]) + Ps[imu_i];
        Vector3d w_pts_2 = Rs[imu_i] * (ric[0] * pts_2 + tic[0]) + Ps[imu_i];
        it_per_id.ptw1 = w_pts_1; // debug use
        it_per_id.ptw2 = w_pts_2;

        // if(isnan(cp(0)))
        {

            // it_per_id.is_triangulation = false;

            // std::cout <<"------------"<<std::endl;
            // std::cout << line << "\n\n";
            // std::cout << d <<"\n\n";
            // std::cout << Rij <<std::endl;
            // std::cout << tij <<"\n\n";
            // std::cout <<"obsj: "<< obsj <<"\n\n";
            // std::cout << "p3: " << p3 <<"\n\n";
            // std::cout << "p4: " << p4 <<"\n\n";
            // std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
            // std::cout << pij <<"\n\n";
        }
    }

    //    removeLineOutlier(Ps,tic,ric);
}

void FeatureManager::removeLineOutlier()
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();

        if (it_per_id->is_triangulation || it_per_id->used_num < 2)
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i - 1;

        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = (p11.cross(p21)).head(2);
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d(0, 0, 0);

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        if (e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }

        if ((e1 - e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }
    }
}

void FeatureManager::removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();
        if (!(it_per_id->used_num >= LINE_MIN_OBS && it_per_id->start_frame < WINDOW_SIZE - 2 && it_per_id->is_triangulation))
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric

        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        //       double  d = nc.norm()/vc.norm();
        //       if (d > 5.0)
        {
            //           std::cerr <<"remove a large distant line \n";
            //           linefeature.erase(it_per_id);
            //           continue;
        }

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = (p11.cross(p21)).head(2);
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d(0, 0, 0);

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        // std::cout << "line endpoint: "<<e1 << "\n "<< e2<<"\n";
        if (e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
        if ((e1 - e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }

        Vector6d line_w = plk_to_pose(it_per_id->line_plucker, Rwc, twc); // transfrom to world frame

        int i = 0;
        double allerr = 0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obs;

        // std::cout<<"reprojection_error: \n";
        for (auto &it_per_frame : it_per_id->linefeature_per_frame)
        {
            imu_j++;

            obs = it_per_frame.lineobs;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            double err = reprojection_error(obs, R1, t1, line_w);

            if (allerr < err)
                allerr = err;
        }
        //        allerr = allerr / i;
        if (allerr > 3.0 / 500.0)
        {
            //            std::cout<<"remove a large error\n";
            linefeature.erase(it_per_id);
        }
    }
}

void FeatureManager::removeBackShiftDepthline(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    // std::cout << "removeBackShiftDepth\n";

    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }

        for (auto it = linefeature.begin(), it_next = linefeature.begin();
             it != linefeature.end(); it = it_next)
        {
            it_next++;

            if (it->start_frame != 0)
            {
                it->start_frame--;
            }
            else
            {

                it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());
                if (it->linefeature_per_frame.size() < 2)
                {
                    linefeature.erase(it);
                    continue;
                }
                else
                {
                    it->removed_cnt++;
                    // transpose this line to the new pose
                    Matrix3d Rji = new_R.transpose() * marg_R; // Rcjw * Rwci
                    Vector3d tji = new_R.transpose() * (marg_P - new_P);
                    Vector6d plk_j = plk_to_pose(it->line_plucker, Rji, tji);
                    it->line_plucker = plk_j;
                }
            }
        }
    }
}