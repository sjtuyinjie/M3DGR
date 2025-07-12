/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"
#include "../factor/pose_subset_parameterization.h"
#include "../factor/orientation_subset_parameterization.h"

#define MAX_GNSS_CAMERA_DELAY 0.1
Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;
    wheelanomaly = false;
    visualstationary = false;
    wheelstationary = false;
    imustationary = false;
    systemstationary = false;
    lowspeed = false;

    varstationary = false;
    preintegrationstationary = false;

    // new
    for (int i = 0; i < WINDOW_SIZE + 1; ++i)
    {
        pre_integrations[i] = nullptr;
        pre_integrations_wheel[i] = nullptr;
    }
    tmp_pre_integration = nullptr;
    tmp_wheel_pre_integration = nullptr;
    last_marginalization_info = nullptr;

    P_imu_tmp[0] = 0, P_imu_tmp[1] = 0, P_imu_tmp[2] = 0;
    P_wheel_tmp[0] = 0, P_wheel_tmp[1] = 0, P_wheel_tmp[2] = 0;
}

void Estimator::clearState()
{
    while (!accBuf.empty()) //
        accBuf.pop();
    while (!gyrBuf.empty())
        gyrBuf.pop();
    while (!featureBuf.empty())
        featureBuf.pop();
    while (!linefeatureBuf.empty())
        linefeatureBuf.pop();
    while (!wheelVelBuf.empty())
        wheelVelBuf.pop();
    while (!wheelGyrBuf.empty())
        wheelGyrBuf.pop();
    while (!GNSSBuf.empty())
        GNSSBuf.pop();

    prevTime = -1;
    prevTime_wheel = -1;
    curTime = 0;
    curTime_wheel = 0;
    openExEstimation = 0;
    openPlaneEstimation = 0;
    openMotionEstimation = 0;
    openExWheelEstimation = 0;
    openIxEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;

        dt_buf_wheel[i].clear();
        linear_velocity_buf_wheel[i].clear();
        angular_velocity_buf_wheel[i].clear();
        if (pre_integrations_wheel[i] != nullptr)
        {
            delete pre_integrations_wheel[i];
        }
        pre_integrations_wheel[i] = nullptr;

        if (GNSS_ENABLE)
            gnss_meas_buf[i].clear();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }
    tio = Vector3d::Zero();
    rio = Matrix3d::Identity();
    sx = 1;
    sy = 1;
    sw = 1;

    rpw = Matrix3d::Identity();
    zpw = 0;

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    first_wheel = false;
    if (tmp_wheel_pre_integration != nullptr)
        delete tmp_wheel_pre_integration;
    tmp_wheel_pre_integration = nullptr;

    gnss_ready = false;
    anc_ecef.setZero();
    R_ecef_enu.setIdentity();
    para_yaw_enu_local[0] = 0;
    yaw_enu_local = 0;
    sat2ephem.clear();
    sat2time_index.clear();
    sat_track_status.clear();
    latest_gnss_iono_params.clear();
    std::copy(GNSS_IONO_DEFAULT_PARAMS.begin(), GNSS_IONO_DEFAULT_PARAMS.end(),
              std::back_inserter(latest_gnss_iono_params));
    diff_t_gnss_local = 0;

    first_optimization = true;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl
             << ric[i] << endl
             << tic[i].transpose() << endl;
    }
    tio = TIO;
    rio = RIO;
    cout << " exitrinsic wheel " << endl
         << rio << endl
         << tio.transpose() << endl;
    sx = SX;
    sy = SY;
    sw = SW;
    vx = 0;
    vy = 0;
    vz = 0;
    cout << " initrinsic wheel " << endl
         << sx << " " << sy << " " << sw << endl;

    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // cout<<"sqrt info:"<<ProjectionTwoFrameOneCamFactor::sqrt_info<<endl;
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    lineProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    td_wheel = TD_WHEEL;
    g = G;
    cout << "set g0 " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES, DEPTH);
    linefeatureTracker.readIntrinsicParameterline(CAM_NAMES[0]);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD)
    {
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;
    TicToc featureTrackerTime;
    if (_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (MULTIPLE_THREAD)
    {
        if (inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}

void Estimator::inputImagebox(double t, const darknet_ros_msgs::BoundingBoxesConstPtr &_boxes, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;
    TicToc featureTrackerTime;
    if (_img1.empty())
        featureFrame = featureTracker.trackImagebox(t, _boxes, _img);
    else
        featureFrame = featureTracker.trackImagebox(t, _boxes, _img, _img1);
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (MULTIPLE_THREAD)
    {
        if (inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}

void Estimator::inputImagewithline(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;
    // map<int, vector<pair<int, Eigen::Matrix<double, 5, 1>>>> linefeatureFrame;
    map<int, vector<pair<int, Vector4d>>> linefeatureFrame;
    TicToc linefeatureTrackerTime;
    if (_img1.empty()) // no depth case
    {
        linefeatureFrame = linefeatureTracker.trackImagewithline(t, _img);
        // ROS_INFO("trackimagewithline");
        featureFrame = featureTracker.trackImage(t, _img);
        // need to bundle them together
    }

    else // with depth
    {
        linefeatureFrame = linefeatureTracker.trackImagewithline(t, _img);
        // cout<<"line feature:"<<linefeatureFrame[0].second(0)<<endl;
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    }
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());   //delete showtrack here

    if (MULTIPLE_THREAD)
    {
        if (inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));

            linefeatureBuf.push(make_pair(t, linefeatureFrame));
            // cout<<"linefeaturebuf size:"<<linefeatureBuf.size()<<endl;
            // cout<<"featurebuf size:"<<featureBuf.size()<<endl;
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));

        linefeatureBuf.push(make_pair(t, linefeatureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    mBuf.unlock();

    fastPredictIMU(t, linearAcceleration, angularVelocity);
    // if (solver_flag == NON_LINEAR)
    {
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        // my new for visualization
        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(t);
        pubIMUPreintegration(latest_P, latest_Q, header);
    }

    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    fastPredictPureIMU(t, linearAcceleration, angularVelocity, p, q, v);
    pubLatestPureOdometry(p, q, v, t);
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    pubPureIMUPreintegration(p, q, header);
}

void Estimator::inputWheel(double t, const Vector3d &linearVelocity, const Vector3d &angularVelocity) // new
{
    mWheelBuf.lock();
    wheelVelBuf.push(make_pair(t, linearVelocity));
    wheelGyrBuf.push(make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    std::ofstream logFile("wheel_log.txt", std::ios::app); // 替换为实际路径
    if (logFile.is_open())
    {
        logFile << std::fixed << std::setprecision(9) // 设置固定格式和小数点后9位
                << "Time: " << t << ", Linear Velocity: " << linearVelocity.transpose() << std::endl;
        logFile.close();
    }

    mWheelBuf.unlock();

    // if (solver_flag == NON_LINEAR)
    {
        mWheelPropagate.lock();
        fastPredictWheel(t, linearVelocity, angularVelocity);
        pubWheelLatestOdometry(latest_P_wheel, latest_Q_wheel, latest_V_wheel, t);

        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(t);
        pubWheelPreintegration(latest_P_wheel, latest_Q_wheel, header);

        // raw
        // pubWheelRawodom(latest_P_wheel, latest_Q_wheel, header);

        Eigen::Quaterniond q;
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        fastPredictPureWheel(t, linearVelocity, angularVelocity, p, q, v);
        pubPureWheelLatestOdometry(p, q, v, t);
        header.frame_id = "world";
        header.stamp = ros::Time(t);
        pubPureWheelPreintegration(p, q, header);
        mWheelPropagate.unlock();
    }
}

void Estimator::inputrawodom(double t, const Vector3d &wheel_xyz) // new
{
    mRawodomBuf.lock();
    wheelxyztBuf.push(make_pair(t, wheel_xyz));

    // printf("input imu with time %f \n", t);
    mRawodomBuf.unlock();
}

void Estimator::inputGNSS(double t, std::vector<ObsPtr> meas_msg) // new
{
    mGNSSBuf.lock();
    GNSSBuf.push(make_pair(t, meas_msg));
    // cout<<"push gnss here"<<endl;

    mGNSSBuf.unlock();
}

bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    // printf("get imu from %f %f\n", t0, t1);
    // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}
bool Estimator::getWheelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &velVector,
                                 vector<pair<double, Eigen::Vector3d>> &gyrVector) // new
{
    if (wheelVelBuf.empty())
    {
        printf("not receive wheel\n");
        return false;
    }
    // printf("get imu from %f %f\n", t0, t1);
    // printf("imu front time %f   imu end time %f\n", wheelVelBuf.front().first, wheelVelBuf.back().first);
    if (t1 <= wheelVelBuf.back().first)
    {
        while (wheelVelBuf.front().first <= t0)
        {
            wheelVelBuf.pop();
            wheelGyrBuf.pop();
        }
        while (wheelVelBuf.front().first < t1)
        {
            velVector.push_back(wheelVelBuf.front());
            wheelVelBuf.pop();
            gyrVector.push_back(wheelGyrBuf.front());
            wheelGyrBuf.pop();
        }
        velVector.push_back(wheelVelBuf.front());
        gyrVector.push_back(wheelGyrBuf.front());
    }
    else
    {
        printf("wait for wheel\n");
        return false;
    }
    //    ROS_INFO("velVector.size: %d", static_cast<int>(velVector.size()));
    return true;
}

bool Estimator::getGNSSInterval(double t0, double t1) // gnss new
{
    if (GNSSBuf.empty())
    {
        // printf("not receive GNSS\n");
        return false;
    }
    // printf("get imu from %f %f\n", t0, t1);
    // printf("imu front time %f   imu end time %f\n", wheelVelBuf.front().first, wheelVelBuf.back().first);

    // time2sec(gnss_meas[0]->time)
    // GNSSBuf.front().first

    while (!GNSSBuf.empty() && time2sec(GNSSBuf.front().second[0]->time) < t1 + diff_t_gnss_local - MAX_GNSS_CAMERA_DELAY)
    {

        ROS_WARN("throw gnss, only should happen at the beginning");
        GNSSBuf.pop();
        if (GNSSBuf.empty())
            return false;
        // front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
    }
    if (GNSSBuf.empty())
    {
        ROS_WARN("wait for gnss...");
        return false;
    }
    else // if (abs(front_gnss_ts-t1-diff_t_gnss_local) < MAX_GNSS_CAMERA_DELAY)
    {

        gnss_msg = GNSSBuf.front().second;
        GNSSBuf.pop();
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}
bool Estimator::WheelAvailable(double t)
{
    // if (!wheelVelBuf.empty())
    // {
    //    // 打印 wheelVelBuf 的大小
    //    std::cout << "wheelVelBuf size: " << wheelVelBuf.size() << std::endl;

    //    // 打印 t 和 wheelVelBuf.back().first 的值
    //    printf("t: %.9f, wheelVelBuf.back().first: %.9f\n", t, wheelVelBuf.back().first);
    // }
    if (!wheelVelBuf.empty() && t <= wheelVelBuf.back().first)
        return true;
    else
        return false;
}
void Estimator::processMeasurements()
{

    while (1)
    {
        // printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>> feature;
        // pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 5, 1>>>>> linefeature;
        pair<double, map<int, vector<pair<int, Vector4d>>>> linefeature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        vector<pair<double, Eigen::Vector3d>> velWheelVector, gyrWheelVector; // new
        
        std::ofstream logFile("output_log.txt", std::ios::app);

        // 打印 featureBuf 的第一个和最后一个时间戳
        if (!featureBuf.empty() || !wheelVelBuf.empty())
        {
            auto feature_first = !featureBuf.empty() ? featureBuf.front().first : 0.0;
            auto feature_last = !featureBuf.empty() ? featureBuf.back().first : 0.0;

            auto wheel_first = !wheelVelBuf.empty() ? wheelVelBuf.front().first : 0.0;
            auto wheel_last = !wheelVelBuf.empty() ? wheelVelBuf.back().first : 0.0;

            logFile << std::fixed << std::setprecision(9) // 设置固定格式和小数点后9位
            << "featureBuf (" << featureBuf.size() << ")- First timestamp: " << feature_first
            << ", Last timestamp: " << feature_last
            << " | wheelVelBuf ("<< wheelVelBuf.size() << ")- First timestamp: " << wheel_first
            << ", Last timestamp: " << wheel_last << std::endl;
        }

        if (!featureBuf.empty())                                              // must happen
        {
            feature = featureBuf.front();

            if (USE_LINE)
            {
                // cout<<"line feature buf:"<<linefeatureBuf.size()<<endl;
                linefeature = linefeatureBuf.front();
            }
            curTime = feature.first + td;
            curTime_wheel = curTime - td_wheel;
            while (1)
            {
                if ((!USE_IMU || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (!MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            while (1)
            {
                if ((!USE_WHEEL || WheelAvailable(feature.first + td - td_wheel)))
                    break;
                else
                {
                    printf("wait for wheel ... \n");
                    if (!MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if (USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            featureBuf.pop();

            if (USE_LINE)
            {
                linefeatureBuf.pop();
            }
            if (GNSS_ENABLE)
            {
                getGNSSInterval(prevTime, curTime);
            }
            mBuf.unlock();
            mWheelBuf.lock();
            if (USE_WHEEL)
                getWheelInterval(prevTime_wheel, curTime_wheel, velWheelVector, gyrWheelVector);
            mWheelBuf.unlock();

            if (USE_IMU)
            {
                dP_imu[0] = 0, dP_imu[1] = 0, dP_imu[2] = 0;
                if (!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            if (USE_WHEEL)
            {
                dP_wheel[0] = 0, dP_wheel[1] = 0, dP_wheel[2] = 0;
                for (size_t i = 0; i < velWheelVector.size(); i++)
                {

                    double dt;

                    if (i == 0)
                        dt = velWheelVector[i].first - prevTime_wheel;
                    else if (i == velWheelVector.size() - 1)
                        dt = curTime_wheel - velWheelVector[i - 1].first;
                    else
                        dt = velWheelVector[i].first - velWheelVector[i - 1].first;

                    processWheel(velWheelVector[i].first, dt, velWheelVector[i].second, gyrWheelVector[i].second);
                }

                double dis = (dP_wheel - dP_imu).norm();//计算距离
                // cout << "imu wheel dis:" << dis << endl;

                // << endl;
                if (dis > 0.02 and wdetect) // 0015 for anomaly good
                {
                    wheelanomaly = true;
                }

                if (dP_wheel.norm() < 0.001)
                { // lower than 0.002

                    // ROS_WARN("Wheel stationary detected! wheel pose norm: %f ", dP_wheel.norm());
                    wheelstationary = true;
                }
                else
                    wheelstationary = false;
                if (dP_imu.norm() < 0.001) // 0.0009
                {                          // lower than 0.002

                    // ROS_WARN("imu preintegration stationary detected! imu pose norm: %f ", dP_imu.norm());
                    preintegrationstationary = true;
                }
                else
                    preintegrationstationary = false;
            }

            if (GNSS_ENABLE && !gnss_msg.empty())
            {

                processGNSS(gnss_msg);
            }
            // line
            if (USE_LINE)
            {
                // cout<<linefeature.second.size();
                processImagewithline(feature.second, feature.first, linefeature.second);
            }
            else
            {
                processImage(feature.second, feature.first); // header
            }

            prevTime = curTime;
            prevTime_wheel = curTime_wheel; // new

            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            pubOdometry(*this, header);

            mVioBuf.lock();
            double t_vio = header.stamp.toSec();
            Vector3d vio_xyz;
            vio_xyz[0] = Ps[WINDOW_SIZE].x();
            vio_xyz[1] = Ps[WINDOW_SIZE].y();
            vio_xyz[2] = Ps[WINDOW_SIZE].z();
            if (solver_flag == NON_LINEAR)
                vioxyztBuf.push(make_pair(t_vio, vio_xyz));

            mVioBuf.unlock(); // odom 20Hz

            Eigen::Matrix<double, 7, 1> pose;
            // pubGroundTruth(*this, header, pose, td);
            pubKeyPoses(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubLinesCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
        }

        if (!MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0 * RIO;
    cout << "init R0 " << endl
         << Rs[0] << endl;
    // Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) // not used
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{

    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        ini_starttime = t;
        cout << "first measurement time:" << ini_starttime << endl;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        //预积分
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // anomaly new

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        // Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        dP_imu = dP_imu + dt * Vs[j] + 0.5 * dt * dt * un_acc;//总距离
        P_imu_tmp += dt * Vs[j] + 0.5 * dt * dt * un_acc;//单次距离
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
void Estimator::processWheel(double t, double dt, const Vector3d &linear_velocity, const Vector3d &angular_velocity) // new
{
    if (!first_wheel)
    {
        first_wheel = true;
        vel_0_wheel = linear_velocity;
        gyr_0_wheel = angular_velocity;
    }

    if (!pre_integrations_wheel[frame_count])
    {
        pre_integrations_wheel[frame_count] = new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};
    }
    if (frame_count != 0)
    {

        pre_integrations_wheel[frame_count]->push_back(dt, linear_velocity, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_wheel_pre_integration->push_back(dt, linear_velocity, angular_velocity);

        dt_buf_wheel[frame_count].push_back(dt);
        linear_velocity_buf_wheel[frame_count].push_back(linear_velocity);
        angular_velocity_buf_wheel[frame_count].push_back(angular_velocity);

        int j = frame_count;

        latest_time_wheel = t;
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0_wheel + angular_velocity);

        Eigen::Vector3d un_vel_0 = Rs[j] * latest_vel_wheel_0;

        Eigen::Matrix3d latest_sv = Eigen::Vector3d(1, 1, 1).asDiagonal();

        // anomaly test
        if (!systemstationary)
        {
            Rs[j] = Rs[j] * Utility::deltaQ(un_gyr * dt); //*RIO.transpose();
            Vs[j] = 0.5 * latest_sv * (Rs[j] * linear_velocity + un_vel_0);
            Ps[j] = Ps[j] + dt * Vs[j]; //*RIO.transpose();
        }
        if (systemstationary)
        {
            Vs[j].setZero();
        }

        latest_vel_wheel_0 = linear_velocity;
        latest_gyr_wheel_0 = angular_velocity;

        dP_wheel[0] -= dt * Vs[j][1];
        dP_wheel[1] += dt * Vs[j][0];
        dP_wheel[2] -= dt * Vs[j][2];

        P_wheel_tmp[0] -= dt * Vs[j][1];
        P_wheel_tmp[1] += dt * Vs[j][0];
        P_wheel_tmp[2] -= dt * Vs[j][2];
    }
    vel_0_wheel = linear_velocity;
    gyr_0_wheel = angular_velocity;
}
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    imageframe.pre_integration_wheel = tmp_wheel_pre_integration; // tmp_wheel_pre_intergration在前面processWheel计算得到
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    tmp_wheel_pre_integration = new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel}; // 重建一个新的，为下一次processWheel做准备

    checkimu();
    if (varstationary && preintegrationstationary)
    {
        imustationary = true;
    }
    else
        imustationary = false;

    // checkvisual

    Matrix3d relative_Rs;
    Vector3d relative_Ts;
    int ls;
    // if(imustationary or wheelstationary)
    if (checkvisual(relative_Rs, relative_Ts, ls))
    {
        // ROS_WARN("Visual stationary detected!");
        visualstationary = true;
    }

    if ((imustationary && wheelstationary) || (visualstationary && wheelstationary) || (imustationary && visualstationary))
    {
        systemstationary = true;
        ROS_WARN("System stationary detected!");
    }
    else
        systemstationary = false;

    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres;
            if (DEPTH)
            {
                corres = f_manager.getCorrespondingWithDepth(frame_count - 1, frame_count);
            }
            else
            {
                corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            }

            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                               << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization     mono version
        if (!(STEREO || DEPTH) && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;
                    cout << "header:" << header << endl;
                }
                if (result)
                {
                    solver_flag = NON_LINEAR;
                    optimization();

                    if (GNSS_ENABLE) // gnss new
                    {
                        if (!gnss_ready)
                        {
                            gnss_ready = GNSSVIAlign();
                        }
                        if (gnss_ready)
                        {
                            updateGNSSStatistics();
                        }
                    }

                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                {
                    optimization(); // no need to gnss vi align
                    slideWindow();
                    updateLatestStates();
                }
            }
        }

        // stereo + IMU initilization    depth version
        else if ((STEREO || DEPTH) && USE_IMU)
        {
            // if (vision_failure)
            // {
            //     f_manager.initFramePoseByOdom(frame_count, Ps, Rs, tic, ric, curTime);
            // }

            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            //  f_manager.triangulate(frame_count, Ps, Rs, tic, ric);//fusion have this
            // rgbd dont have this two
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    // result = 1; //visualInitialAlignWithDepth();
                    result = initialStructure();
                    initial_timestamp = header;
                }
                // if (vision_failure)
                // {
                //     result = true;
                // }

                if (result)
                {

                    // solveGyroscopeBias(all_image_frame, Bgs);
                    for (int i = 0; i <= WINDOW_SIZE; i++)
                    {
                        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                    }
                    solver_flag = NON_LINEAR;

                    optimization();

                    if (GNSS_ENABLE) // gnss new
                    {
                        if (!gnss_ready)
                        {
                            gnss_ready = GNSSVIAlign();
                        }
                        if (gnss_ready)
                        {
                            updateGNSSStatistics();
                        }
                    }

                    slideWindow();
                    // ROS_WARN("Visual Initialization finish!");
                }
                else
                {
                    optimization();
                    slideWindow();
                    updateLatestStates();
                }
            }
        }

        // stereo only initilization
        else if ((STEREO || DEPTH) && !USE_IMU) // no imu
        {
            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            if (DEPTH)
            {
                f_manager.triangulateWithDepth(frame_count, Ps, Rs, tic, ric);
            }
            if (USE_LINE)
            {
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
                f_manager.triangulateLine(frame_count, Ps, Rs, tic, ric);
            }
            else
            {
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            }

            optimization();

            if (frame_count == WINDOW_SIZE)
            {
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if (frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

        if (solver_flag == NON_LINEAR && USE_PLANE)
            initPlane();
    }
    else // not ini
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        // f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        // if (vision_failure)
        // {
        //     f_manager.initFramePoseByOdom(frame_count, Ps, Rs, tic, ric, curTime);
        // }
        if (DEPTH)
        {
            f_manager.triangulateWithDepth(frame_count, Ps, Rs, tic, ric);
        }
        if (USE_LINE)
        {
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulateLine(frame_count, Ps, Rs, tic, ric);
        }
        else
        {
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        }

        set<int> removeIndex;
        // outliersRejection(removeIndex);
        if (USE_MCC)
        {
            movingConsistencyCheckW(removeIndex);
            f_manager.removeOutlier(removeIndex);
        }

        optimization();
        if (GNSS_ENABLE)
        {
            if (!gnss_ready)
            {
                gnss_ready = GNSSVIAlign();
            }
            if (gnss_ready)
            {
                updateGNSSStatistics();
            }
        }
        if (!USE_MCC)
        {
            set<int> removeIndex;
            // outliersRejection(removeIndex);
            movingConsistencyCheckW(removeIndex);
            f_manager.removeOutlier(removeIndex);
        }

        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

// line
void Estimator::processImagewithline(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const double header, const map<int, vector<pair<int, Vector4d>>> &linefeature)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    ROS_INFO("Adding feature lines %lu", linefeature.size());

    if (f_manager.addFeatureCheckParallaxwithline(frame_count, image, linefeature, td))
    {
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    imageframe.pre_integration_wheel = tmp_wheel_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    tmp_wheel_pre_integration = new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};

    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres;
            if (DEPTH)
            {
                corres = f_manager.getCorrespondingWithDepth(frame_count - 1, frame_count);
            }
            else
            {
                corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            }

            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                               << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization     mono version
        if (!(STEREO || DEPTH) && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;
                }
                if (result)
                {
                    solver_flag = NON_LINEAR;
                    if (USE_LINE)
                    {
                        onlyLineOpt();
                        // optimizationwithLine();
                        optimization();
                    }
                    else
                    {
                        optimization();
                    }

                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization    depth version
        else if ((STEREO || DEPTH) && USE_IMU) // this is the case  depth  ini
        {
            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            //  f_manager.triangulate(frame_count, Ps, Rs, tic, ric);//fusion have this
            // rgbd dont have this two
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    // result = 1; //visualInitialAlignWithDepth();
                    result = initialStructure();
                    initial_timestamp = header;
                }

                if (result)
                {

                    solveGyroscopeBias(all_image_frame, Bgs);
                    for (int i = 0; i <= WINDOW_SIZE; i++)
                    {
                        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                    }
                    solver_flag = NON_LINEAR;
                    if (USE_LINE)
                    {
                        onlyLineOpt();
                        optimizationwithLine();
                        // optimization();
                    }
                    else
                    {
                        optimization();
                    }
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                {
                    slideWindow();
                }
                // new init begin here
            }
        }

        // stereo only initilization
        else if ((STEREO || DEPTH) && !USE_IMU) // no imu
        {
            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            if (DEPTH)
            {
                f_manager.triangulateWithDepth(frame_count, Ps, Rs, tic, ric);
            }
            if (USE_LINE)
            {
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
                f_manager.triangulateLine(frame_count, Ps, Rs, tic, ric);
            }
            else
            {
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            }
            if (USE_LINE)
            {
                onlyLineOpt();
                // optimizationwithLine();
                optimization();
            }
            else
            {
                optimization();
            }

            if (frame_count == WINDOW_SIZE)
            {
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if (frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

        if (solver_flag == NON_LINEAR && USE_PLANE)
            initPlane();
    }
    else // not ini
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        // f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        if (DEPTH)
        {
            f_manager.triangulateWithDepth(frame_count, Ps, Rs, tic, ric);
        }

        if (USE_LINE)
        {
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulateLine(frame_count, Ps, Rs, tic, ric);
        }
        else
        {
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        }
        if (USE_LINE)
        {
            onlyLineOpt();
            optimizationwithLine();
            // optimization();
        }
        else
        {
            optimization();
        }
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

// gnss new
void Estimator::inputEphem(EphemBasePtr ephem_ptr)
{
    double toe = time2sec(ephem_ptr->toe);
    // if a new ephemeris comes
    if (sat2time_index.count(ephem_ptr->sat) == 0 || sat2time_index.at(ephem_ptr->sat).count(toe) == 0)
    {
        sat2ephem[ephem_ptr->sat].emplace_back(ephem_ptr);
        sat2time_index[ephem_ptr->sat].emplace(toe, sat2ephem.at(ephem_ptr->sat).size() - 1);
    }
}

void Estimator::inputIonoParams(double ts, const std::vector<double> &iono_params)
{
    if (iono_params.size() != 8)
        return;

    // update ionosphere parameters
    latest_gnss_iono_params.clear();
    std::copy(iono_params.begin(), iono_params.end(), std::back_inserter(latest_gnss_iono_params));
}

void Estimator::inputGNSSTimeDiff(const double t_diff)
{
    diff_t_gnss_local = t_diff;
}

void Estimator::processGNSS(const std::vector<ObsPtr> &gnss_meas)
{
    // cout<<"here2"<<endl;
    std::vector<ObsPtr> valid_meas;
    std::vector<EphemBasePtr> valid_ephems;
    for (auto obs : gnss_meas)
    {
        // filter according to system
        uint32_t sys = satsys(obs->sat, NULL);
        if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS)
            continue;

        // if not got cooresponding ephemeris yet
        if (sat2ephem.count(obs->sat) == 0)
            continue;

        if (obs->freqs.empty())
            continue; // no valid signal measurement
        int freq_idx = -1;
        L1_freq(obs, &freq_idx);
        if (freq_idx < 0)
            continue; // no L1 observation

        double obs_time = time2sec(obs->time);
        std::map<double, size_t> time2index = sat2time_index.at(obs->sat);
        double ephem_time = EPH_VALID_SECONDS;
        size_t ephem_index = -1;
        for (auto ti : time2index)
        {
            if (std::abs(ti.first - obs_time) < ephem_time)
            {
                ephem_time = std::abs(ti.first - obs_time);
                ephem_index = ti.second;
            }
        }
        if (ephem_time >= EPH_VALID_SECONDS)
        {
            cerr << "ephemeris not valid anymore\n";
            continue;
        }
        const EphemBasePtr &best_ephem = sat2ephem.at(obs->sat).at(ephem_index);

        // filter by tracking status
        LOG_IF(FATAL, freq_idx < 0) << "No L1 observation found.\n";
        if (obs->psr_std[freq_idx] > GNSS_PSR_STD_THRES ||
            obs->dopp_std[freq_idx] > GNSS_DOPP_STD_THRES)
        {
            sat_track_status[obs->sat] = 0;
            continue;
        }
        else
        {
            if (sat_track_status.count(obs->sat) == 0)
                sat_track_status[obs->sat] = 0;
            ++sat_track_status[obs->sat];
        }
        if (sat_track_status[obs->sat] < GNSS_TRACK_NUM_THRES)
            continue; // not being tracked for enough epochs

        // filter by elevation angle
        if (gnss_ready)
        {
            Eigen::Vector3d sat_ecef;
            if (sys == SYS_GLO)
                sat_ecef = geph2pos(obs->time, std::dynamic_pointer_cast<GloEphem>(best_ephem), NULL);
            else
                sat_ecef = eph2pos(obs->time, std::dynamic_pointer_cast<Ephem>(best_ephem), NULL);
            double azel[2] = {0, M_PI / 2.0};
            sat_azel(ecef_pos, sat_ecef, azel);
            if (azel[1] < GNSS_ELEVATION_THRES * M_PI / 180.0)
                continue;
        }
        valid_meas.push_back(obs);
        valid_ephems.push_back(best_ephem);
    }
    cout << "valid satellite num:" << valid_meas.size() << endl; //<<endl<<endl<<endl;
   

    gnss_meas_buf[frame_count] = valid_meas;
    gnss_ephem_buf[frame_count] = valid_ephems;
}

void Estimator::initPlane()
{ // new
    double zpws = 0.0;
    std::vector<Eigen::Quaterniond> qpws;
    for (int i = 0; i < frame_count; ++i)
    {
        Eigen::Matrix3d rpw_temp = (Rs[i] * rio).transpose(); //   roation in wheel frame
        qpws.emplace_back(Eigen::Quaterniond(rpw_temp));
        zpws += -(rpw_temp * (Ps[i] + Rs[i] * tio))[2]; // change in z in wheel frame
    }
    Eigen::Quaterniond qpw = Utility::quaternionAverage(qpws);
    rpw = qpw.toRotationMatrix();

    double yaw = Utility::R2ypr(rpw).x();
    rpw = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * rpw;
    zpw = zpws / frame_count;
    std::cout << "Init plane:  rpw: " << Eigen::AngleAxisd(rpw).axis().transpose() << " zpw: " << zpw << std::endl;
}

int ini_cnt = 0;
bool Estimator::initialStructure()
{
    ini_cnt += 1;
    TicToc t_sfm;
    // bool is_imu_excited = false;
    Vector3d aver_g;
    // check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }

        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;

            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // cout << "frame g " << tmp_g.transpose() << endl;
            // cout << "aver g " << aver_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        // ROS_WARN("IMU variation %f!", var);
        // cout << "cnt num:" << ini_cnt << endl;
        if (var < 0.35) // 0.35
        // if(cnt>20)
        {
            // is_imu_excited = true;

            ROS_INFO("IMU excitation not enouth! var: %f", var);
            // return false;
        }
        else // 0.25
        {
            is_imu_excited = true;
            ROS_WARN("IMU excitation OK! var: %f", var);
            // is_imu_excited=false;
        }
    }

    if (Bas_calibok == false and systemstationary and solver_flag != NON_LINEAR)
    {

        Vector3d tmp_Bas = aver_g - Utility::g2R(aver_g).inverse() * G;
        cout << "aver g:" << aver_g << endl;
        cout << "G:" << g << endl;
        ROS_WARN_STREAM("accelerator bias initial calibration " << tmp_Bas.transpose());
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Bas[i] = tmp_Bas;
        }

        Bas_calibok = true;
        solveGyroscopeBias(all_image_frame, Bgs);
        // align g
        {
            Matrix3d R0 = Utility::g2R(g);
            Vector3d ypr = Utility::R2ypr(R0);
            
            // 仅保留X轴和Y轴方向（重置航向角）
            ypr[0] = 0; // 重置YAW角
            Matrix3d R_init = Utility::ypr2R(ypr);
            
            // 使用对齐后的重力方向更新重力矢量
            g = R_init * g; 

            // 初始化所有状态（姿态、位置、速度）
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Rs[i] = R_init;
                Ps[i].setZero();
                Vs[i].setZero();
                Bas[i] = tmp_Bas; // 确保零偏一致
                // Bgs[i] 在solveGyroscopeBias中已初始化
            }
        }
        
        ROS_WARN("Stationary initialization successful!");
        // solver_flag = NON_LINEAR;

        return true;
    }

    if (Bas_calibok == false and is_imu_excited)
    {

        Vector3d tmp_Bas = aver_g - Utility::g2R(aver_g).inverse() * G;
        cout << "aver g:" << aver_g << endl;
        cout << "G:" << g << endl;
        ROS_WARN_STREAM("accelerator bias initial calibration " << tmp_Bas.transpose());
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Bas[i] = tmp_Bas;
        }

        Bas_calibok = true;
        solveGyroscopeBias(all_image_frame, Bgs);
        // align g
        // {
        //     Matrix3d R0 = Utility::g2R(g);
        //     double yaw = 0;

        //     yaw = Utility::R2ypr(R0 * Rs[0]).x();
        //     double pitch = Utility::R2ypr(R0 * Rs[0]).y();
        //     double roll = Utility::R2ypr(R0 * Rs[0]).z();

        //     R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, -pitch, -roll}) * R0; // set yaw to zero
        //     g = R0 * g;

        //     Matrix3d rot_diff = R0; // org

        //     for (int i = 0; i <= frame_count; i++)
        //     {
        //         Ps[i] = rot_diff * Ps[i];
        //         Rs[i] = rot_diff * Rs[i];
        //         Vs[i] = rot_diff * Vs[i];
        //     }
        // }
        ROS_WARN("Wheel activation initialization successful!");
        return true;
    }
    // is_imu_excited=false;

    // if (is_imu_excited)
    // {
    //     return false;
    // }
    // return false;

    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            tmp_feature.observation_depth.push_back(make_pair(imu_j, it_per_frame.depth));
            // cout<<"it perframe depth:"<<it_per_frame.depth<<endl;
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (DEPTH && !relativePoseWithDepth(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features; Move device around(Depth version)");
        return false;
    }
    else if (!DEPTH && !relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around(no depth versioin)");
        return false;
    }
    GlobalSFM sfm;
    if (DEPTH)
    {
        if (!sfm.constructWithDepth(frame_count + 1, Q, T, l,
                                    relative_R, relative_T,
                                    sfm_f, sfm_tracked_points))
        {
            ROS_DEBUG("global SFM failed!");
            marginalization_flag = MARGIN_OLD;
            return false;
        }
    }
    else
    {
        if (!sfm.construct(frame_count + 1, Q, T, l,
                           relative_R, relative_T,
                           sfm_f, sfm_tracked_points))
        {
            ROS_DEBUG("global SFM failed!");
            marginalization_flag = MARGIN_OLD;
            return false;
        }
    }

    // solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
    {
        // if (STATIONARY_INI)
        {

            //            map<double, ImageFrame>::iterator frame_it;
            //            Vector3d sum_a(0, 0, 0);
            //            for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
            //                double dt = frame_it->second.pre_integration->sum_dt;
            //                Vector3d tmp_a = frame_it->second.pre_integration->delta_v / dt;
            //                sum_a += tmp_a;
            //            }
            //            Vector3d avg_a;
            //            avg_a = sum_a * 1.0 / ((int) all_image_frame.size() - 1);

            Vector3d tmp_Bas = aver_g - Utility::g2R(aver_g).inverse() * G;
            ROS_WARN_STREAM("accelerator bias initial calibration " << tmp_Bas.transpose());
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Bas[i] = tmp_Bas;
            }
        }
        ROS_WARN("Visual initialization successful!");
        return true;
    }

    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    // solve scale
     // 1.初始化陀螺仪bias; 初始化速度, 重力和尺度
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        ROS_INFO("solve g failed!");
        return false;
    }
    // 2.change state 滑动窗口内keyframes的PVQ
    // change state传递所有图像帧的位姿Ps、Rs，并将其置为关键帧
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    
    //缺少重新三角化

    double s = (x.tail<1>())(0);//尺度
    //重新预积分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    //相机坐标系 --> 世界坐标系
    Matrix3d R0 = Utility::g2R(g);
    double yaw = 0;

    yaw = Utility::R2ypr(R0 * Rs[0]).x();

    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; // set yaw to zero
    g = R0 * g;
    // cout<<Rs[0].transpose()<<endl;//almost identity
    // exit(-1);
    //  Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0; // org

    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    f_manager.clearDepth();
    if (DEPTH)
    {
        f_manager.triangulateWithDepth(frame_count, Ps, Rs, tic, ric);
    }
    // if (USE_LINE)//not this in plvins
    if (0)
    {
        f_manager.triangulateLine(frame_count, Ps, Rs, tic, ric);
    }
    else
    {
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
    }
    // f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::GNSSVIAlign()
{

    // if (solver_flag == INITIAL )//&& is_imu_excited == false) // visual-inertial not initialized
    //     return false;

    if (is_imu_excited == false && solver_flag == INITIAL)
    {
        cout << "perform gnss vi align: imu not excited or initial" << endl;
        return false;
    }

    if (gnss_ready) // GNSS-VI already initialized
    {
        cout << "perform gnss vi align: gnss already true" << endl;
        return true;
    }

    // for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
    // {
    //     if (gnss_meas_buf[i].empty() )
    //     //if (gnss_meas_buf[i].empty() || gnss_meas_buf[i].size() < 2)
    //         return false;
    // }

    // check horizontal velocity excitation
    Eigen::Vector2d avg_hor_vel(0.0, 0.0);
    for (uint32_t i = 0; i < (WINDOW_SIZE + 1); ++i)
        avg_hor_vel += Vs[i].head<2>().cwiseAbs();
    avg_hor_vel /= (WINDOW_SIZE + 1);
    if (avg_hor_vel.norm() < 0.3)
    {
        std::cerr << "velocity excitation not enough for GNSS-VI alignment.\n";
        return false;
    }

    std::vector<std::vector<ObsPtr>> curr_gnss_meas_buf;
    std::vector<std::vector<EphemBasePtr>> curr_gnss_ephem_buf;
    for (uint32_t i = 0; i < (WINDOW_SIZE + 1); ++i)
    {
        curr_gnss_meas_buf.push_back(gnss_meas_buf[i]);
        curr_gnss_ephem_buf.push_back(gnss_ephem_buf[i]);
    }

    GNSSVIInitializer gnss_vi_initializer(curr_gnss_meas_buf, curr_gnss_ephem_buf, latest_gnss_iono_params);

    // 1. get a rough global location
    Eigen::Matrix<double, 7, 1> rough_xyzt;
    rough_xyzt.setZero();
    if (!gnss_vi_initializer.coarse_localization(rough_xyzt))
    {
        std::cerr << "Fail to obtain a coarse location.\n";
        return false;
    }
    // cout << "rough xyzt" << rough_xyzt << endl
    //      << endl
    //      << endl;

    // 2. perform yaw alignment
    std::vector<Eigen::Vector3d> local_vs;
    for (uint32_t i = 0; i < (WINDOW_SIZE + 1); ++i)
        local_vs.push_back(Vs[i]);
    Eigen::Vector3d rough_anchor_ecef = rough_xyzt.head<3>();
    double aligned_yaw = 0;
    double aligned_rcv_ddt = 0;
    if (!gnss_vi_initializer.yaw_alignment(local_vs, rough_anchor_ecef, aligned_yaw, aligned_rcv_ddt))
    {
        std::cerr << "Fail to align ENU and local frames.\n";
        return false;
    }
    std::cout << "aligned_yaw is " << aligned_yaw * 180.0 / M_PI << '\n';

    // 3. perform anchor refinement
    std::vector<Eigen::Vector3d> local_ps;
    for (uint32_t i = 0; i < (WINDOW_SIZE + 1); ++i)
        local_ps.push_back(Ps[i]);
    Eigen::Matrix<double, 7, 1> refined_xyzt;
    refined_xyzt.setZero();
    if (!gnss_vi_initializer.anchor_refinement(local_ps, aligned_yaw,
                                               aligned_rcv_ddt, rough_xyzt, refined_xyzt))
    {
        std::cerr << "Fail to refine anchor point.\n";
        return false;
    }
    // std::cout << "refined anchor point is " << std::setprecision(20)
    //           << refined_xyzt.head<3>().transpose() << '\n';

    // restore GNSS states
    uint32_t one_observed_sys = static_cast<uint32_t>(-1);
    for (uint32_t k = 0; k < 4; ++k)
    {
        if (rough_xyzt(k + 3) != 0)
        {
            one_observed_sys = k;
            break;
        }
    }
    for (uint32_t i = 0; i < (WINDOW_SIZE + 1); ++i)
    {
        para_rcv_ddt[i] = aligned_rcv_ddt;
        for (uint32_t k = 0; k < 4; ++k)
        {
            if (rough_xyzt(k + 3) == 0)
                para_rcv_dt[i * 4 + k] = refined_xyzt(3 + one_observed_sys) + aligned_rcv_ddt * i;
            else
                para_rcv_dt[i * 4 + k] = refined_xyzt(3 + k) + aligned_rcv_ddt * i;
        }
    }
    anc_ecef = refined_xyzt.head<3>();
    R_ecef_enu = ecef2rotation(anc_ecef);

    yaw_enu_local = aligned_yaw;
    ROS_WARN("GNSS VI Align ok!");

    return true;
}

void Estimator::updateGNSSStatistics()
{
    R_enu_local = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ());
    // cout<<"yaw enu local:"<<yaw_enu_local<<endl;
    enu_pos = R_enu_local * Ps[WINDOW_SIZE];
    enu_vel = R_enu_local * Vs[WINDOW_SIZE];
    enu_ypr = Utility::R2ypr(R_enu_local * Rs[WINDOW_SIZE]);
    // cout<<"enu_pos:"<<std::setprecision(9)<<enu_pos<<endl;
    // cout<<"anc ecef pos"<<anc_ecef<<endl;
    ecef_pos = anc_ecef + R_ecef_enu * enu_pos;
    // cout<<"trajectory ecef_pos:"<<std::setprecision(9)<<anc_ecef<<endl;
}
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

bool Estimator::relativePoseWithDepth(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        // corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        corres = f_manager.getCorrespondingWithDepth(i, WINDOW_SIZE);
        // cout<<"corres size:"<<corres.size()<<endl;
        if (corres.size() > 20)
        {
            if (m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T)) // && STATIONARY_INI)
            {
                l = i;
                return true;
            }

            // cout<<"corres size:"<<corres.size()<<endl;
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0) / corres[j].first(2), corres[j].first(1) / corres[j].first(2));
                Vector2d pts_1(corres[j].second(0) / corres[j].second(2), corres[j].second(1) / corres[j].second(2));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            // cout<<"avg parallax:"<<average_parallax<<endl;
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T))
            {
                //                Matrix3d relative_R2; Vector3d relative_T2;
                //                m_estimator.solveRelativeRT(corres, relative_R2, relative_T2);
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::checkimuexcited()
{
    Vector3d aver_g;
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        sum_g += tmp_g;
    }

    aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        // cout<<"imu delta v g:"<<tmp_g<<endl;
        // Vector3d tmp_gw = frame_it->second.pre_integration_wheel->delta_v / dt;
        // cout<<"wheel delta v:"<<tmp_gw<<endl;

        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        // cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int)all_image_frame.size() - 1));
    // ROS_WARN("IMU variation %f!", var);
    // cout << "cnt num:" << ini_cnt << endl;
    if (var < 0.35) // 035
    // if(cnt>20)
    {
        // is_imu_excited = true;

        ROS_INFO("IMU excitation not enouth: %f", var);
        // return false;
    }
    else // 0.25
    {
        is_imu_excited = true;
        ROS_WARN("IMU excitation OK!");
        // is_imu_excited=false;
    }
}

void Estimator::checkimu()
{
    Vector3d aver_g;
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        sum_g += tmp_g;
    }

    aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        // cout<<"imu delta v g:"<<tmp_g<<endl;
        // Vector3d tmp_gw = frame_it->second.pre_integration_wheel->delta_v / dt;
        // cout<<"wheel delta v:"<<tmp_gw<<endl;

        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        // cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int)all_image_frame.size() - 1));
    // ROS_WARN("IMU variation %f!", var);
    // cout << "cnt num:" << ini_cnt << endl;
    // cout<<"var check:"<<var<<endl<<endl;

  
    if (var < 0.1)
    // if(cnt>20)
    {
        // is_imu_excited = true;
        varstationary = true;

        // ROS_WARN("IMU var stationary detected!");
        //  return false;
    }
    else // 0.25
    {
        varstationary = false;
        // ROS_WARN("IMU excitation OK!");
        // is_imu_excited=false;
    }
}

bool Estimator::checkvisual(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        // corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        corres = f_manager.getCorrespondingWithDepth(i, WINDOW_SIZE);
        // cout<<"corres size:"<<corres.size()<<endl;
        if (corres.size() > 20)
        {
            // if (m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T) && STATIONARY_INI)
            // {
            //     l = i;
            //     return true;
            // }

            // cout<<"corres size:"<<corres.size()<<endl;
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0) / corres[j].first(2), corres[j].first(1) / corres[j].first(2));
                Vector2d pts_1(corres[j].second(0) / corres[j].second(2), corres[j].second(1) / corres[j].second(2));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());

            // cout<<"avg parallax:"<<average_parallax*460<<endl;
            //  }
            //  if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T))
            //  {
            //      //                Matrix3d relative_R2; Vector3d relative_T2;
            //      //                m_estimator.solveRelativeRT(corres, relative_R2, relative_T2);
            //      l = i;
            //      ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
            //      return true;
            //  }
            if (average_parallax * 460 < 0.5 && m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T)) // org 30
            {
                //                Matrix3d relative_R2; Vector3d relative_T2;
                //                m_estimator.solveRelativeRT(corres, relative_R2, relative_T2);
                l = i;

                // ROS_WARN("Visual stationary detected! average_parallax %f ", average_parallax * 460);
                return true;
            }
            visualstationary = false;
        }
        visualstationary = false;
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    para_Ex_Pose_wheel[0][0] = tio.x(); // new
    para_Ex_Pose_wheel[0][1] = tio.y();
    para_Ex_Pose_wheel[0][2] = tio.z();
    Quaterniond q{rio};
    para_Ex_Pose_wheel[0][3] = q.x();
    para_Ex_Pose_wheel[0][4] = q.y();
    para_Ex_Pose_wheel[0][5] = q.z();
    para_Ex_Pose_wheel[0][6] = q.w();

    para_Ix_sx_wheel[0][0] = sx;
    para_Ix_sy_wheel[0][0] = sy;
    para_Ix_sw_wheel[0][0] = sw;

    Quaterniond q2{rpw};
    para_plane_R[0][0] = q2.x();
    para_plane_R[0][1] = q2.y();
    para_plane_R[0][2] = q2.z();
    para_plane_R[0][3] = q2.w();
    para_plane_Z[0][0] = zpw;

    para_motion[0][0] = vx;
    para_motion[0][1] = vy;
    para_motion[0][2] = vz;

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
    para_Td_wheel[0][0] = td_wheel;
    if (gnss_ready)
    {
        para_yaw_enu_local[0] = yaw_enu_local;
        for (uint32_t k = 0; k < 3; ++k)
            para_anc_ecef[k] = anc_ecef(k);
    }
}

void Estimator::vector2doubleline()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    para_Ex_Pose_wheel[0][0] = tio.x(); // new
    para_Ex_Pose_wheel[0][1] = tio.y();
    para_Ex_Pose_wheel[0][2] = tio.z();
    Quaterniond q{rio};
    para_Ex_Pose_wheel[0][3] = q.x();
    para_Ex_Pose_wheel[0][4] = q.y();
    para_Ex_Pose_wheel[0][5] = q.z();
    para_Ex_Pose_wheel[0][6] = q.w();

    para_Ix_sx_wheel[0][0] = sx;
    para_Ix_sy_wheel[0][0] = sy;
    para_Ix_sw_wheel[0][0] = sw;

    Quaterniond q2{rpw};
    para_plane_R[0][0] = q2.x();
    para_plane_R[0][1] = q2.y();
    para_plane_R[0][2] = q2.z();
    para_plane_R[0][3] = q2.w();
    para_plane_Z[0][0] = zpw;

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
    para_Td_wheel[0][0] = td_wheel;

#ifdef LINEINCAM
    MatrixXd lineorth = f_manager.getLineOrthVectorInCamera();
#else
    MatrixXd lineorth = f_manager.getLineOrthVector(Ps, tic, ric);
#endif

    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i)
    { // line
        para_LineFeature[i][0] = lineorth.row(i)[0];
        para_LineFeature[i][1] = lineorth.row(i)[1];
        para_LineFeature[i][2] = lineorth.row(i)[2];
        para_LineFeature[i][3] = lineorth.row(i)[3];
        if (i > NUM_OF_F)
            std::cerr << " 1000  1000 1000 1000 1000 \n\n";
    }
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                         para_Pose[0][3],
                                                         para_Pose[0][4],
                                                         para_Pose[0][5])
                                                 .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();

        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .toRotationMatrix();
        }
    }

    if (USE_WHEEL)
    {
        tio = Vector3d(para_Ex_Pose_wheel[0][0],
                       para_Ex_Pose_wheel[0][1],
                       para_Ex_Pose_wheel[0][2]);
        rio = Quaterniond(para_Ex_Pose_wheel[0][6],
                          para_Ex_Pose_wheel[0][3],
                          para_Ex_Pose_wheel[0][4],
                          para_Ex_Pose_wheel[0][5])
                  .normalized()
                  .toRotationMatrix();

        // cout<<"tio:"<<tio<<endl;
        // cout<<"rio:"<<rio<<endl;
        sx = para_Ix_sx_wheel[0][0];
        sy = para_Ix_sy_wheel[0][0];
        sw = para_Ix_sw_wheel[0][0];

        td_wheel = para_Td_wheel[0][0];
    }

    if (USE_PLANE)
    {
        zpw = para_plane_Z[0][0];
        rpw = Quaterniond(para_plane_R[0][3], para_plane_R[0][0], para_plane_R[0][1], para_plane_R[0][2]).normalized().toRotationMatrix();
    }
    if (USE_MOTION)
    {
        vx = para_motion[0][0];
        vy = para_motion[0][1];
        vz = para_motion[0][2];
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];

    if (gnss_ready) // gnss new
    {
        yaw_enu_local = para_yaw_enu_local[0];
        for (uint32_t k = 0; k < 3; ++k)
            anc_ecef(k) = para_anc_ecef[k];
        R_ecef_enu = ecef2rotation(anc_ecef);
    }
}

void Estimator::double2vectorline()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                         para_Pose[0][3],
                                                         para_Pose[0][4],
                                                         para_Pose[0][5])
                                                 .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();

        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .toRotationMatrix();
        }
    }

    if (USE_WHEEL)
    {
        tio = Vector3d(para_Ex_Pose_wheel[0][0],
                       para_Ex_Pose_wheel[0][1],
                       para_Ex_Pose_wheel[0][2]);
        rio = Quaterniond(para_Ex_Pose_wheel[0][6],
                          para_Ex_Pose_wheel[0][3],
                          para_Ex_Pose_wheel[0][4],
                          para_Ex_Pose_wheel[0][5])
                  .normalized()
                  .toRotationMatrix();
        sx = para_Ix_sx_wheel[0][0];
        sy = para_Ix_sy_wheel[0][0];
        sw = para_Ix_sw_wheel[0][0];

        td_wheel = para_Td_wheel[0][0];
    }

    if (USE_PLANE)
    {
        zpw = para_plane_Z[0][0];
        rpw = Quaterniond(para_plane_R[0][3], para_plane_R[0][0], para_plane_R[0][1], para_plane_R[0][2]).normalized().toRotationMatrix();
    }

    // line below
    // std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(), 4); // line
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i)
    {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);
        lineorth_vec.row(i) = orth;
    }
#ifdef LINEINCAM
    f_manager.setLineOrthInCamera(lineorth_vec);
#else
    f_manager.setLineOrth(lineorth_vec, Ps, Rs, tic, ric);
#endif
    // above line

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];
}

void Estimator::double2vector2()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();

    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0)); // move from use imu

    if (USE_IMU)
    {

        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .toRotationMatrix();
        }
    }

    if (USE_WHEEL)
    {
        tio = Vector3d(para_Ex_Pose_wheel[0][0],
                       para_Ex_Pose_wheel[0][1],
                       para_Ex_Pose_wheel[0][2]);
        rio = Quaterniond(para_Ex_Pose_wheel[0][6],
                          para_Ex_Pose_wheel[0][3],
                          para_Ex_Pose_wheel[0][4],
                          para_Ex_Pose_wheel[0][5])
                  .normalized()
                  .toRotationMatrix();
        sx = para_Ix_sx_wheel[0][0];
        sy = para_Ix_sy_wheel[0][0];
        sw = para_Ix_sw_wheel[0][0];

        td_wheel = para_Td_wheel[0][0];
    }

    if (USE_PLANE)
    {
        zpw = para_plane_Z[0][0];
        rpw = Quaterniond(para_plane_R[0][3], para_plane_R[0][0], para_plane_R[0][1], para_plane_R[0][2]).normalized().toRotationMatrix();
    }

    if (USE_MOTION)
    {
        vx = para_motion[0][0];
        vy = para_motion[0][1];
        vz = para_motion[0][2];
    }

    Matrix3d Rwow1 = rot_diff;
    Vector3d tw1b(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2]);
    Vector3d twow1 = -Rwow1 * tw1b + origin_P0;

    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(), 4);
    ;
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i)
    {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);

        Vector6d line_w1 = orth_to_plk(orth);
        Vector6d line_wo = plk_to_pose(line_w1, Rwow1, twow1);
        orth = plk_to_orth(line_wo);

        lineorth_vec.row(i) = orth;
    }
    f_manager.setLineOrth(lineorth_vec, Ps, Rs, tic, ric);
    // line above
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }

    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        // return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        // ROS_INFO(" big z translation");
        // return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0); // original
    // loss_function = new ceres::CauchyLoss(1.0);//for test

    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    if (gnss_ready) // gnss new
    {
        problem.AddParameterBlock(para_yaw_enu_local, 1);
        Eigen::Vector2d avg_hor_vel(0.0, 0.0);
        for (uint32_t i = 0; i <= WINDOW_SIZE; ++i)
            avg_hor_vel += Vs[i].head<2>().cwiseAbs();
        avg_hor_vel /= (WINDOW_SIZE + 1);
        // cerr << "avg_hor_vel is " << avg_vel << endl;
        if (avg_hor_vel.norm() < 0.3) // org 0.3
        {
            // std::cerr << "velocity excitation not enough, fix yaw angle.\n";
            // problem.SetParameterBlockConstant(para_yaw_enu_local);
            cout << "too small velocity, fix yaw angle" << endl;
            lowspeed = true;
        }
        else
            lowspeed = false;

        // for (uint32_t i = 0; i <= WINDOW_SIZE; ++i)
        // {
        //     if (gnss_meas_buf[i].size() < 10) // org 10
        //     {
        //         problem.SetParameterBlockConstant(para_yaw_enu_local);
        //         cout << "gnss meas < 10, fix yaw angle" << endl;
        //     }
        // }
        problem.SetParameterBlockConstant(para_yaw_enu_local);

        problem.AddParameterBlock(para_anc_ecef, 3);
        // problem.SetParameterBlockConstant(para_anc_ecef);

        for (uint32_t i = 0; i <= WINDOW_SIZE; ++i)
        {
            for (uint32_t k = 0; k < 4; ++k)
                problem.AddParameterBlock(para_rcv_dt + i * 4 + k, 1);
            problem.AddParameterBlock(para_rcv_ddt + i, 1);
        }
    }

    if (first_optimization && GNSS_ENABLE) // gnss new
    {
        std::vector<double> anchor_value;
        for (uint32_t k = 0; k < 7; ++k)
            anchor_value.push_back(para_Pose[0][k]);
        PoseAnchorFactor *pose_anchor_factor = new PoseAnchorFactor(anchor_value);
        problem.AddResidualBlock(pose_anchor_factor, NULL, para_Pose[0]);
        first_optimization = false;
    }

    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++) // new
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        if (ESTIMATE_EXTRINSIC)
        {
            // 固定某些量
            switch (CAM_EXT_ADJ_TYPE)
            {
            case CameraExtrinsicAdjustType::ADJUST_CAM_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 6});
                break;
            case CameraExtrinsicAdjustType::ADJUST_CAM_ROTATION:
                local_parameterization = new PoseSubsetParameterization({0, 1, 2, 6});
                break;
            case CameraExtrinsicAdjustType::ADJUST_CAM_TRANSLATION:
                local_parameterization = new PoseSubsetParameterization({3, 4, 5, 6});
                break;
            case CameraExtrinsicAdjustType::ADJUST_CAM_NO_ROTATION_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 3, 4, 5, 6});
                break;
            default:
                local_parameterization = new PoseSubsetParameterization({});
            }
        }
        else
            local_parameterization = new PoseLocalParameterization(); // new
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);//这个变量固定为constant
        }
    }

    if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) // && is_imu_excited) //(solver_flag == NON_LINEAR ||is_imu_excited)
    {                                          // new
        //        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();

        ceres::LocalParameterization *local_parameterization;
        if (ESTIMATE_EXTRINSIC_WHEEL)
        {
            // 固定某些量
            switch (WHEEL_EXT_ADJ_TYPE)
            {
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 6});
                break;
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_ROTATION:
                local_parameterization = new PoseSubsetParameterization({0, 1, 2, 6});
                break;
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_TRANSLATION:
                local_parameterization = new PoseSubsetParameterization({3, 4, 5, 6});
                break;
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_ROTATION_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 3, 4, 5, 6});
                break;
            default:
                local_parameterization = new PoseSubsetParameterization({});
            }
        }
        else
            local_parameterization = new PoseLocalParameterization();

        problem.AddParameterBlock(para_Ex_Pose_wheel[0], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExWheelEstimation)
        {
            // ROS_INFO("estimate extrinsic param");
            openExWheelEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose_wheel[0]);
        }
        problem.AddParameterBlock(para_Ix_sx_wheel[0], 1);
        problem.AddParameterBlock(para_Ix_sy_wheel[0], 1);
        problem.AddParameterBlock(para_Ix_sw_wheel[0], 1);
        if ((ESTIMATE_INTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openIxEstimation)
        {
            // ROS_INFO("estimate intrinsic param");
            openIxEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ix_sx_wheel[0]);
            problem.SetParameterBlockConstant(para_Ix_sy_wheel[0]);
            problem.SetParameterBlockConstant(para_Ix_sw_wheel[0]);
        }
    }

    if (USE_PLANE)
    {
        ceres::LocalParameterization *local_parameterization = new OrientationSubsetParameterization(std::vector<int>{2});
        problem.AddParameterBlock(para_plane_R[0], SIZE_ROTATION, local_parameterization);
        problem.AddParameterBlock(para_plane_Z[0], 1);
        if (frame_count == WINDOW_SIZE || openPlaneEstimation)
        {
            // ROS_INFO("estimate extrinsic param");
            openPlaneEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_plane_R[0]);
            problem.SetParameterBlockConstant(para_plane_Z[0]);
        }
    }

    if (USE_MOTION)
    {
        // ceres::LocalParameterization *local_parameterization = new OrientationSubsetParameterization(std::vector<int>{2});
        problem.AddParameterBlock(para_motion[0], 3);

        if (frame_count == WINDOW_SIZE || openMotionEstimation)
        {

            openMotionEstimation = 1;
        }
        else
        {

            problem.SetParameterBlockConstant(para_motion[0]);
        }
    }

    problem.AddParameterBlock(para_Td[0], 1);//相机和imu时间戳同步
    problem.AddParameterBlock(para_Td_wheel[0], 1); // new wheel和imu时间戳同步

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);
    if (!ESTIMATE_TD_WHEEL || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td_wheel[0]); // new

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) // && is_imu_excited) //(solver_flag == NON_LINEAR ||is_imu_excited)
    {
        for (int i = 0; i < frame_count; i++)
        {
            // cout<< "use wheel now"<<endl<<endl<<endl;
            int j = i + 1;
            if (pre_integrations_wheel[j]->sum_dt > 10.0) // 两图像帧之间时间过长，不使用中间的预积分
            {
                ROS_INFO("too long between two wheel frames, skip!");
                continue;
            }

            if (wdetect && wheelanomaly)
            {
                ROS_WARN("wheel anomaly, skip optimization!");
                continue;
            }

            WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[j]);
            problem.AddResidualBlock(wheel_factor, NULL, para_Pose[i], para_Pose[j], para_Ex_Pose_wheel[0], para_Ix_sx_wheel[0], para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]);

            //            std::vector<const double *> parameters(7);
            //            parameters[0] = para_Pose[i];
            //            parameters[1] = para_Pose[j];
            //            parameters[2] = para_Ex_Pose_wheel[0];
            //            parameters[3] = para_Ix_sx_wheel[0];
            //            parameters[4] = para_Ix_sy_wheel[0];
            //            parameters[5] = para_Ix_sw_wheel[0];
            //            parameters[6] = para_Td_wheel[0];
            //            wheel_factor->check(const_cast<double **>(parameters.data()));
        }
    }

    if (USE_PLANE)
    {
        for (int i = 0; i < frame_count; i++)
        {
            PlaneFactor *plane_factor = new PlaneFactor();
            problem.AddResidualBlock(plane_factor, NULL, para_Pose[i], para_Ex_Pose_wheel[0], para_plane_R[0], para_plane_Z[0]);

            //            std::vector<const double *> parameters(4);
            //            parameters[0] = para_Pose[i];
            //            parameters[1] = para_Ex_Pose_wheel[0];
            //            parameters[2] = para_plane_R[0];
            //            parameters[3] = para_plane_Z[0];
            //            plane_factor->check(const_cast<double **>(parameters.data()));
        }
    }

    if (USE_MOTION)
    {
        for (int i = 0; i < frame_count; i++)
        {
            MotionFactor *motion_factor = new MotionFactor();
            problem.AddResidualBlock(motion_factor, NULL, para_Pose[i], para_Ex_Pose_wheel[0], para_SpeedBias[i], para_motion[0]);
        }
    }

    if (gnss_ready and !lowspeed) // gnss new
    {
        // if (!lowspeed){
        {
            for (int i = 0; i <= WINDOW_SIZE; ++i)
            {
                // cerr << "size of gnss_meas_buf[" << i << "] is " << gnss_meas_buf[i].size() << endl;
                const std::vector<ObsPtr> &curr_obs = gnss_meas_buf[i];
                const std::vector<EphemBasePtr> &curr_ephem = gnss_ephem_buf[i];

                for (uint32_t j = 0; j < curr_obs.size(); ++j)
                {
                    const uint32_t sys = satsys(curr_obs[j]->sat, NULL);
                    const uint32_t sys_idx = gnss_comm::sys2idx.at(sys);

                    int lower_idx = -1;
                    const double obs_local_ts = time2sec(curr_obs[j]->time) - diff_t_gnss_local;
                    if (Headers[i] > obs_local_ts)
                        lower_idx = (i == 0 ? 0 : i - 1);
                    else
                        lower_idx = (i == WINDOW_SIZE ? WINDOW_SIZE - 1 : i);
                    const double lower_ts = Headers[lower_idx];
                    const double upper_ts = Headers[lower_idx + 1];

                    const double ts_ratio = (upper_ts - obs_local_ts) / (upper_ts - lower_ts);
                    GnssPsrDoppFactor *gnss_factor = new GnssPsrDoppFactor(curr_obs[j],
                                                                           curr_ephem[j], latest_gnss_iono_params, ts_ratio);
                    problem.AddResidualBlock(gnss_factor, NULL, para_Pose[lower_idx],
                                             para_SpeedBias[lower_idx], para_Pose[lower_idx + 1], para_SpeedBias[lower_idx + 1],
                                             para_rcv_dt + i * 4 + sys_idx, para_rcv_ddt + i, para_yaw_enu_local, para_anc_ecef);
                }
            }
        }

        // build relationship between rcv_dt and rcv_ddt
        for (size_t k = 0; k < 4; ++k)
        {
            for (uint32_t i = 0; i < WINDOW_SIZE; ++i)
            {
                const double gnss_dt = Headers[i + 1] - Headers[i];
                DtDdtFactor *dt_ddt_factor = new DtDdtFactor(gnss_dt);
                problem.AddResidualBlock(dt_ddt_factor, NULL, para_rcv_dt + i * 4 + k,
                                         para_rcv_dt + (i + 1) * 4 + k, para_rcv_ddt + i, para_rcv_ddt + i + 1);
            }
        }

        // add rcv_ddt smooth factor
        for (int i = 0; i < WINDOW_SIZE; ++i)
        {
            DdtSmoothFactor *ddt_smooth_factor = new DdtSmoothFactor(GNSS_DDT_WEIGHT);
            problem.AddResidualBlock(ddt_smooth_factor, NULL, para_rcv_ddt + i, para_rcv_ddt + i + 1);
        }
    }

    // new stationary
    if (systemstationary && stationary_detect)
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            para_SpeedBias[i][0] = 0;
            para_SpeedBias[i][1] = 0;
            para_SpeedBias[i][2] = 0;

            problem.SetParameterBlockConstant(para_Pose[i]);
            problem.SetParameterBlockConstant(para_SpeedBias[i]);
            // problem.SetParameterBlockConstant(para_SpeedBias[i][0]);
            // problem.SetParameterBlockConstant(para_SpeedBias[i][1]);
            // problem.SetParameterBlockConstant(para_SpeedBias[i][2]);
        }
        // cout<<"velocit now"<<para_SpeedBias[9][0]<<endl<<endl<<endl;
        // cout<<"pose"<<para_Pose[9]<<endl;

        // problem.SetParameterBlockConstant(para_Pose[0]);
        // problem.SetParameterBlockConstant(para_Pose[0]);

        // Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        // Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
        //                             para_Pose[i][1] - para_Pose[0][1],
        //                             para_Pose[i][2] - para_Pose[0][2]) +
        //         origin_P0;

        // Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
        //                             para_SpeedBias[i][1],
        //                             para_SpeedBias[i][2]);
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    ProjectionTwoFrameTwoCamFactor::sqrt_info = f_m_cnt / 600 * ProjectionTwoFrameTwoCamFactor::sqrt_info;
    // 600
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // cout<<"used num:"<<it_per_id.feature_per_frame.size()<<endl;
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        // 被首帧观测到的归一化坐标 (第一个归一化坐标)
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        // 遍历观测到该feature的所有frame
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                if (it_per_id.estimate_flag == 1)
                    problem.SetParameterBlockConstant(para_Feature[feature_index]); // new
            }

            f_m_cnt++;
        }
    }
    ROS_INFO("visual measurement count: %d", f_m_cnt);

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());
    while (para_yaw_enu_local[0] > M_PI)
        para_yaw_enu_local[0] -= 2.0 * M_PI;
    while (para_yaw_enu_local[0] < -M_PI)
        para_yaw_enu_local[0] += 2.0 * M_PI;

    double2vector();
    // printf("frame_count: %d \n", frame_count);

    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) // && is_imu_excited) //(solver_flag == NON_LINEAR ||is_imu_excited)
        {
            if (pre_integrations_wheel[1]->sum_dt < 10.0 && !(wdetect && wheelanomaly))
            {
                WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(wheel_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_Pose[1], para_Ex_Pose_wheel[0], para_Ix_sx_wheel[0], para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]},
                                                                               vector<int>{0}); // 边缘化 para_Pose[0]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        if (USE_PLANE)
        {
            PlaneFactor *plane_factor = new PlaneFactor();
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(plane_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_Ex_Pose_wheel[0], para_plane_R[0], para_plane_Z[0]},
                                                                           vector<int>{0}); // 边缘化 para_Pose[0]
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // if (USE_MOTION)
        // {
        //     MotionFactor *motion_factor = new MotionFactor();
        //     ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(motion_factor, NULL,
        //                                                                    vector<double *>{para_Pose[0], para_Ex_Pose_wheel[0], para_SpeedBias[0], para_motion[0]},
        //                                                                    vector<int>{0}); // 边缘化 para_Pose[0]
        //     marginalization_info->addResidualBlockInfo(residual_block_info);
        // }

        if (gnss_ready) // gnss new
        {
            for (uint32_t j = 0; j < gnss_meas_buf[0].size(); ++j)
            {
                const uint32_t sys = satsys(gnss_meas_buf[0][j]->sat, NULL);
                const uint32_t sys_idx = gnss_comm::sys2idx.at(sys);

                const double obs_local_ts = time2sec(gnss_meas_buf[0][j]->time) - diff_t_gnss_local;
                const double lower_ts = Headers[0];
                const double upper_ts = Headers[1];
                const double ts_ratio = (upper_ts - obs_local_ts) / (upper_ts - lower_ts);

                GnssPsrDoppFactor *gnss_factor = new GnssPsrDoppFactor(gnss_meas_buf[0][j],
                                                                       gnss_ephem_buf[0][j], latest_gnss_iono_params, ts_ratio);
                ResidualBlockInfo *psr_dopp_residual_block_info = new ResidualBlockInfo(gnss_factor, NULL,
                                                                                        vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                                                                                                         para_SpeedBias[1], para_rcv_dt + sys_idx, para_rcv_ddt,
                                                                                                         para_yaw_enu_local, para_anc_ecef},
                                                                                        vector<int>{0, 1, 4, 5});
                marginalization_info->addResidualBlockInfo(psr_dopp_residual_block_info);
            }

            const double gnss_dt = Headers[1] - Headers[0]; // duplicate parameter block
            for (size_t k = 0; k < 4; ++k)
            {
                DtDdtFactor *dt_ddt_factor = new DtDdtFactor(gnss_dt);
                ResidualBlockInfo *dt_ddt_residual_block_info = new ResidualBlockInfo(dt_ddt_factor, NULL,
                                                                                      vector<double *>{para_rcv_dt + k, para_rcv_dt + 4 + k, para_rcv_ddt, para_rcv_ddt + 1},
                                                                                      vector<int>{0, 2});
                marginalization_info->addResidualBlockInfo(dt_ddt_residual_block_info);
            }

            // margin rcv_ddt smooth factor Null pointer passed to AddParameterBlock for a parameter with size 1
            DdtSmoothFactor *ddt_smooth_factor = new DdtSmoothFactor(GNSS_DDT_WEIGHT);
            ResidualBlockInfo *ddt_smooth_residual_block_info = new ResidualBlockInfo(ddt_smooth_factor, NULL,
                                                                                      vector<double *>{para_rcv_ddt, para_rcv_ddt + 1}, vector<int>{0});
            marginalization_info->addResidualBlockInfo(ddt_smooth_residual_block_info);
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;
                // 被首帧观测到的归一化坐标 (第一个归一化坐标)
                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    // if ((DEPTH || STEREO) && it_per_frame.is_stereo)
                    // {
                    //     Vector3d pts_j_right = it_per_frame.pointRight;
                    //     if (imu_i != imu_j)
                    //     {
                    //         ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    //                                                                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    //         ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                    //                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                    //                                                                        vector<int>{0, 4});
                    //         marginalization_info->addResidualBlockInfo(residual_block_info);
                    //     }
                    //     else
                    //     {
                    //         ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    //                                                                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    //         ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                    //                                                                        vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                    //                                                                        vector<int>{2});
                    //         marginalization_info->addResidualBlockInfo(residual_block_info);
                    //     }
                    // }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());
        // 10.1.6.调整参数块在下一次窗口中对应的位置（往前移一格），注意这里是指针，后面slideWindow中会赋新值，这里只是提前占座
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            for (uint32_t k = 0; k < 4; ++k)
                addr_shift[reinterpret_cast<long>(para_rcv_dt + i * 4 + k)] = para_rcv_dt + (i - 1) * 4 + k;
            addr_shift[reinterpret_cast<long>(para_rcv_ddt + i)] = para_rcv_ddt + i - 1;
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
        addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])] = para_Ix_sx_wheel[0];
        addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])] = para_Ix_sy_wheel[0];
        addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])] = para_Ix_sw_wheel[0];

        addr_shift[reinterpret_cast<long>(para_plane_R[0])] = para_plane_R[0];
        addr_shift[reinterpret_cast<long>(para_plane_Z[0])] = para_plane_Z[0];

        addr_shift[reinterpret_cast<long>(para_Td_wheel[0])] = para_Td_wheel[0];
        if (GNSS_ENABLE)
        {
            addr_shift[reinterpret_cast<long>(para_yaw_enu_local)] = para_yaw_enu_local;
            addr_shift[reinterpret_cast<long>(para_anc_ecef)] = para_anc_ecef;
        }

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        // ----------------------10.2.边缘化次新帧-----------------------
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            else
            {
                std::vector<double> anchor_value;
                for (uint32_t k = 0; k < 7; ++k)
                    anchor_value.push_back(para_Pose[0][k]);
                PoseAnchorFactor *pose_anchor_factor = new PoseAnchorFactor(anchor_value);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(pose_anchor_factor,
                                                                               NULL, vector<double *>{para_Pose[0]}, vector<int>{0});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {

                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    for (uint32_t k = 0; k < 4; ++k)
                        addr_shift[reinterpret_cast<long>(para_rcv_dt + i * 4 + k)] = para_rcv_dt + (i - 1) * 4 + k;
                    addr_shift[reinterpret_cast<long>(para_rcv_ddt + i)] = para_rcv_ddt + i - 1;
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    for (uint32_t k = 0; k < 4; ++k)
                        addr_shift[reinterpret_cast<long>(para_rcv_dt + i * 4 + k)] = para_rcv_dt + i * 4 + k;
                    addr_shift[reinterpret_cast<long>(para_rcv_ddt + i)] = para_rcv_ddt + i;
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])] = para_Ix_sx_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])] = para_Ix_sy_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])] = para_Ix_sw_wheel[0];

            addr_shift[reinterpret_cast<long>(para_plane_R[0])] = para_plane_R[0];
            addr_shift[reinterpret_cast<long>(para_plane_Z[0])] = para_plane_Z[0];
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            addr_shift[reinterpret_cast<long>(para_Td_wheel[0])] = para_Td_wheel[0];

            if (GNSS_ENABLE)
            {
                addr_shift[reinterpret_cast<long>(para_yaw_enu_local)] = para_yaw_enu_local;
                addr_shift[reinterpret_cast<long>(para_anc_ecef)] = para_anc_ecef;
            }

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }

    wheelanomaly = false;
    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    // printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if (USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
                if (USE_WHEEL)
                {
                    std::swap(pre_integrations_wheel[i], pre_integrations_wheel[i + 1]);

                    dt_buf_wheel[i].swap(dt_buf_wheel[i + 1]);
                    linear_velocity_buf_wheel[i].swap(linear_velocity_buf_wheel[i + 1]);
                    angular_velocity_buf_wheel[i].swap(angular_velocity_buf_wheel[i + 1]);
                }

                if (GNSS_ENABLE)
                {
                    gnss_meas_buf[i].swap(gnss_meas_buf[i + 1]);
                    gnss_ephem_buf[i].swap(gnss_ephem_buf[i + 1]);
                    for (uint32_t k = 0; k < 4; ++k)
                        para_rcv_dt[i * 4 + k] = para_rcv_dt[(i + 1) * 4 + k];
                    para_rcv_ddt[i] = para_rcv_ddt[i + 1];
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            if (GNSS_ENABLE) // gnss new
            {
                gnss_meas_buf[WINDOW_SIZE].clear();
                gnss_ephem_buf[WINDOW_SIZE].clear();
            }
            if (USE_WHEEL)
            {

                delete pre_integrations_wheel[WINDOW_SIZE];
                pre_integrations_wheel[WINDOW_SIZE] = new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};

                dt_buf_wheel[WINDOW_SIZE].clear();
                linear_velocity_buf_wheel[WINDOW_SIZE].clear();
                angular_velocity_buf_wheel[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                delete it_0->second.pre_integration_wheel; // new
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if (USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            // GNSS related
            gnss_meas_buf[frame_count - 1] = gnss_meas_buf[frame_count];
            gnss_ephem_buf[frame_count - 1] = gnss_ephem_buf[frame_count];
            for (uint32_t k = 0; k < 4; ++k)
                para_rcv_dt[(frame_count - 1) * 4 + k] = para_rcv_dt[frame_count * 4 + k];
            para_rcv_ddt[frame_count - 1] = para_rcv_ddt[frame_count];
            gnss_meas_buf[frame_count].clear();
            gnss_ephem_buf[frame_count].clear();

            if (USE_WHEEL)
            {
                for (unsigned int i = 0; i < dt_buf_wheel[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf_wheel[frame_count][i];
                    Vector3d tmp_linear_velocity = linear_velocity_buf_wheel[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf_wheel[frame_count][i];

                    pre_integrations_wheel[frame_count - 1]->push_back(tmp_dt, tmp_linear_velocity, tmp_angular_velocity);

                    dt_buf_wheel[frame_count - 1].push_back(tmp_dt);
                    linear_velocity_buf_wheel[frame_count - 1].push_back(tmp_linear_velocity);
                    angular_velocity_buf_wheel[frame_count - 1].push_back(tmp_angular_velocity);
                }

                delete pre_integrations_wheel[WINDOW_SIZE];
                pre_integrations_wheel[WINDOW_SIZE] = new WheelIntegrationBase{vel_0_wheel, gyr_0_wheel, sx, sy, sw, td_wheel};

                dt_buf_wheel[WINDOW_SIZE].clear();
                linear_velocity_buf_wheel[WINDOW_SIZE].clear();
                angular_velocity_buf_wheel[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    if (USE_LINE)
    {
        f_manager.removeFrontline(frame_count);
    }
    else
        f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        if (USE_LINE)
        {
            f_manager.removeBackShiftDepthline(R0, P0, R1, P1);
        }
        else
            f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
    {
        if (USE_LINE)
        {
            f_manager.removeBackline();
        }
        else
            f_manager.removeBack();
    }
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    // printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            // printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

double Estimator::reprojectionError3D(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                      Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                      double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    return (pts_cj - uvj).norm() / depth;
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    // return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                     depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            // if ((DEPTH || STEREO) && it_per_frame.is_stereo)
            // {

            //     Vector3d pts_j_right = it_per_frame.pointRight;
            //     if (imu_i != imu_j)
            //     {
            //         double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
            //                                              Rs[imu_j], Ps[imu_j], ric[1], tic[1],
            //                                              depth, pts_i, pts_j_right);
            //         err += tmp_error;
            //         errCnt++;
            //         //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            //     }
            //     else
            //     {
            //         double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
            //                                              Rs[imu_j], Ps[imu_j], ric[1], tic[1],
            //                                              depth, pts_i, pts_j_right);
            //         err += tmp_error;
            //         errCnt++;
            //         //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            //     }
            // }
        }
        double ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

void Estimator::movingConsistencyCheckW(set<int> &removeIndex)
{
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        double depth = it_per_id.estimated_depth;
        if (depth < 0)
            continue;

        double err = 0;
        double err3D = 0;
        int errCnt = 0;
        int wheel_i = it_per_id.start_frame, wheel_j = wheel_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            wheel_j++;
            if (wheel_i != wheel_j)
            {
                Vector3d pts_j = it_per_frame.point;
                err += reprojectionError(Rs[wheel_i], Ps[wheel_i], ric[0], tic[0], Rs[wheel_j], Ps[wheel_j],
                                         ric[0], tic[0], depth, pts_i, pts_j);
                // for bleeding points
                err3D += reprojectionError3D(Rs[wheel_i], Ps[wheel_i], ric[0], tic[0], Rs[wheel_j],
                                             Ps[wheel_j], ric[0], tic[0], depth, pts_i, pts_j);
                errCnt++;
            }
        }
        if (errCnt > 0)
        {
            if (FOCAL_LENGTH * err / errCnt > 10 || err3D / errCnt > 2.0)
            {
                removeIndex.insert(it_per_id.feature_id);
                // it_per_id.is_dynamic = true;
            }
            // else
            // {
            //     it_per_id.is_dynamic = false;
            // }
        }
    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);       //中值积分
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;

    // cout<<"p1 change:"<<dt * latest_V + 0.5 * dt * dt * un_acc<<endl;
    // cout<<"v1 change:"<<dt * un_acc<<endl;
}

void Estimator::fastPredictPureIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity, Eigen::Vector3d &P, Eigen::Quaterniond &Q, Eigen::Vector3d &V)
{
    static bool first_timex = false;
    static Eigen::Quaterniond Q_latestx(1, 0, 0, 0);
    static Eigen::Vector3d V_latestx = Eigen::Vector3d::Zero();
    static Eigen::Vector3d P_latestx = Eigen::Vector3d::Zero();
    static Eigen::Vector3d acc_0x = Eigen::Vector3d::Zero();
    static Eigen::Vector3d gyr_0x = Eigen::Vector3d::Zero();
    static double t_latestx;
    if (!first_timex)
    {
        first_timex = true;
        Q_latestx = latest_Q;
        V_latestx = latest_V;
        P_latestx = latest_P;
        acc_0x = latest_acc_0;
        gyr_0x = latest_gyr_0;
        t_latestx = latest_time;
        std::cout << "fastPredictPureimu initial pose: \n"
                  << P_latestx.transpose() << std::endl
                  << Q_latestx.coeffs().transpose() << std::endl;
    }

    double dt = t - t_latestx;
    t_latestx = t;
    Eigen::Vector3d un_acc_0 = Q_latestx * (acc_0x - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0x + angular_velocity) - latest_Bg;
    Q_latestx = Q_latestx * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = Q_latestx * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    P_latestx = P_latestx + dt * V_latestx + 0.5 * dt * dt * un_acc;
    V_latestx = V_latestx + dt * un_acc;
    // cout<<"v latest:"<<V_latestx<<endl;
    // cout<<"un acc:"<<un_acc<<endl;
    // cout<<"dt:"<<dt<<endl;
    // cout<<"v dt:"<<dt * V_latestx<<endl;
    // cout<<"p change:"<<dt * V_latestx + 0.5 * dt * dt * un_acc<<endl;
    // cout<<"v change:"<<dt * un_acc<<endl;
    acc_0x = linear_acceleration;
    gyr_0x = angular_velocity;

    P = P_latestx;
    Q = Q_latestx;
    V = V_latestx;
}

// 中值积分Wheel航迹解算new
void Estimator::fastPredictWheel(double t, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time_wheel;
    latest_time_wheel = t;
    Eigen::Vector3d un_gyr = 0.5 * latest_sw * (latest_gyr_0 + angular_velocity);
    Eigen::Vector3d un_vel_0 = latest_Q_wheel * latest_vel_wheel_0;
    Eigen::Matrix3d latest_sv = Eigen::Vector3d(latest_sx, latest_sy, 1).asDiagonal();

    latest_Q_wheel = latest_Q_wheel * Utility::deltaQ(un_gyr * dt);
    latest_V_wheel = 0.5 * latest_sv * (latest_Q_wheel * linear_velocity + un_vel_0);

    latest_P_wheel = latest_P_wheel + dt * latest_V_wheel;
    latest_vel_wheel_0 = linear_velocity;
    latest_gyr_wheel_0 = angular_velocity;
}

void Estimator::fastPredictPureWheel(double t, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity, Eigen::Vector3d &P, Eigen::Quaterniond &Q, Eigen::Vector3d &V)
{
    static bool first_time = false;
    static Eigen::Quaterniond Q_latest(1, 0, 0, 0);
    static Eigen::Vector3d V_latest = Eigen::Vector3d::Zero();
    static Eigen::Vector3d P_latest = Eigen::Vector3d::Zero();
    static Eigen::Vector3d vel_latest_0 = Eigen::Vector3d::Zero();
    static Eigen::Vector3d gyr_latest_0 = Eigen::Vector3d::Zero();
    static double t_latest;
    if (!first_time)
    {
        first_time = true;
        // Q_latest = latest_Q_wheel;
        V_latest = latest_V_wheel;
        P_latest = latest_P_wheel;
        vel_latest_0 = latest_vel_wheel_0;
        gyr_latest_0 = latest_gyr_wheel_0;
        t_latest = latest_time_wheel;
        std::cout << "fastPredictPureWheel initial pose: \n"
                  << P_latest.transpose() << std::endl
                  << Q_latest.coeffs().transpose() << std::endl;
    }

    double dt = t - t_latest;
    t_latest = t;
    Eigen::Vector3d un_gyr = 0.5 * latest_sw * (gyr_latest_0 + angular_velocity);
    Eigen::Vector3d un_vel_0 = Q_latest * vel_latest_0;

    // cout<<"q latest"<<Q_latest.coeffs()<<endl;
    // cout<<"vel latest 0:"<<vel_latest_0<<endl;//only x
    // cout<<"un vel:"<<un_vel_0<<endl;
    Eigen::Matrix3d latest_sv = Eigen::Vector3d(latest_sx, latest_sy, 1).asDiagonal();

    Q_latest = Q_latest * Utility::deltaQ(un_gyr * dt);
    V_latest = 0.5 * latest_sv * (Q_latest * linear_velocity + un_vel_0);
    P_latest = P_latest + dt * V_latest;
    // cout<<"wheel velocity:"<<V_latest<<endl;//always vz zero
    //  cout<<P_latest<<endl<<V_latest<<endl;
    vel_latest_0 = linear_velocity;
    gyr_latest_0 = angular_velocity;

    P = P_latest;
    Q = Q_latest;
    V = V_latest;
}

void Estimator::updateLatestStates()
{
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    // cout<<"local v latest"<<latest_V<<endl;
    // cout<<"imu velocity latest:"<<Rs[frame_count].transpose()*latest_V<<endl;//*rio<<end;
    // cout<<"wheel velocity latest:"<<rio.transpose()*Rs[frame_count].transpose()*latest_V<<endl;//*rio<<end;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    while (!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mBuf.unlock();

    mWheelPropagate.lock();
    latest_time_wheel = Headers[frame_count] + td - td_wheel;
    // latest_Q_wheel = Rs[frame_count] * RIO;
    // latest_P_wheel = Rs[frame_count] * TIO + Ps[frame_count];
    latest_Q_wheel = Rs[frame_count] * rio;
    latest_P_wheel = Rs[frame_count] * tio + Ps[frame_count];
    latest_sx = sx;
    latest_sy = sy;
    latest_sw = sw;
    // cout<<rio<<endl;
    // cout<<tio<<endl;
    // latest_V_wheel可以在fastPredictWheel算出来，不需要初值
    //     latest_V_wheel = Vs[frame_count];
    latest_vel_wheel_0 = vel_0_wheel;
    latest_gyr_wheel_0 = gyr_0_wheel;
    mWheelBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_wheel_velBuf = wheelVelBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_wheel_gyrBuf = wheelGyrBuf;
    mWheelBuf.unlock();

    while (!tmp_wheel_velBuf.empty())
    {
        double t = tmp_wheel_velBuf.front().first;
        Eigen::Vector3d vel = tmp_wheel_velBuf.front().second;
        Eigen::Vector3d gyr = tmp_wheel_gyrBuf.front().second;
        fastPredictWheel(t, vel, gyr);
        tmp_wheel_velBuf.pop();
        tmp_wheel_gyrBuf.pop();
    }
    mWheelPropagate.unlock();
}

// line below

void Estimator::onlyLineOpt()
{

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization); // p,q

        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    vector2doubleline();

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        ++feature_index;

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock(para_LineFeature[feature_index], SIZE_LINE, local_parameterization_line); // p,q

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            Vector4d obs = it_per_frame.lineobs;
            lineProjectionFactor *f = new lineProjectionFactor(obs);
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[feature_index]);
            f_m_cnt++;
        }
    }

    if (feature_index < 3)
    {
        return;
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double2vectorline();
    // std::cout << summary.FullReport()<<std::endl;

    f_manager.removeLineOutlier(Ps, tic, ric);
}

void Estimator::optimizationwithLine()
{
    TicToc t_whole, t_prepare;
    vector2doubleline();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++) // new
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        if (ESTIMATE_EXTRINSIC)
        {

            switch (CAM_EXT_ADJ_TYPE)
            {
            case CameraExtrinsicAdjustType::ADJUST_CAM_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 6});
                break;
            case CameraExtrinsicAdjustType::ADJUST_CAM_ROTATION:
                local_parameterization = new PoseSubsetParameterization({0, 1, 2, 6});
                break;
            case CameraExtrinsicAdjustType::ADJUST_CAM_TRANSLATION:
                local_parameterization = new PoseSubsetParameterization({3, 4, 5, 6});
                break;
            case CameraExtrinsicAdjustType::ADJUST_CAM_NO_ROTATION_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 3, 4, 5, 6});
                break;
            default:
                local_parameterization = new PoseSubsetParameterization({});
            }
        }
        else
            local_parameterization = new PoseLocalParameterization(); // new
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {

            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }

    if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) // && is_imu_excited) //(solver_flag == NON_LINEAR ||is_imu_excited)
    {                                          // new
        //        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();

        ceres::LocalParameterization *local_parameterization;
        if (ESTIMATE_EXTRINSIC_WHEEL)
        {

            switch (WHEEL_EXT_ADJ_TYPE)
            {
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 6});
                break;
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_ROTATION:
                local_parameterization = new PoseSubsetParameterization({0, 1, 2, 6});
                break;
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_TRANSLATION:
                local_parameterization = new PoseSubsetParameterization({3, 4, 5, 6});
                break;
            case WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_ROTATION_NO_Z:
                local_parameterization = new PoseSubsetParameterization({2, 3, 4, 5, 6});
                break;
            default:
                local_parameterization = new PoseSubsetParameterization({});
            }
        }
        else
            local_parameterization = new PoseLocalParameterization();

        problem.AddParameterBlock(para_Ex_Pose_wheel[0], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExWheelEstimation)
        {

            openExWheelEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose_wheel[0]);
        }
        problem.AddParameterBlock(para_Ix_sx_wheel[0], 1);
        problem.AddParameterBlock(para_Ix_sy_wheel[0], 1);
        problem.AddParameterBlock(para_Ix_sw_wheel[0], 1);
        if ((ESTIMATE_INTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openIxEstimation)
        {
            // ROS_INFO("estimate intrinsic param");
            openIxEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ix_sx_wheel[0]);
            problem.SetParameterBlockConstant(para_Ix_sy_wheel[0]);
            problem.SetParameterBlockConstant(para_Ix_sw_wheel[0]);
        }
    }

    if (USE_PLANE)
    {
        ceres::LocalParameterization *local_parameterization = new OrientationSubsetParameterization(std::vector<int>{2});
        problem.AddParameterBlock(para_plane_R[0], SIZE_ROTATION, local_parameterization);
        problem.AddParameterBlock(para_plane_Z[0], 1);
        if (frame_count == WINDOW_SIZE || openPlaneEstimation)
        {
            // ROS_INFO("estimate extrinsic param");
            openPlaneEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_plane_R[0]);
            problem.SetParameterBlockConstant(para_plane_Z[0]);
        }
    }

    problem.AddParameterBlock(para_Td[0], 1);
    problem.AddParameterBlock(para_Td_wheel[0], 1); // new

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);
    if (!ESTIMATE_TD_WHEEL || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td_wheel[0]); // new

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) // && is_imu_excited) // (solver_flag == NON_LINEAR ||is_imu_excited)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations_wheel[j]->sum_dt > 10.0)
                continue;

            WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[j]);
            problem.AddResidualBlock(wheel_factor, NULL, para_Pose[i], para_Pose[j], para_Ex_Pose_wheel[0], para_Ix_sx_wheel[0], para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]);

            //            std::vector<const double *> parameters(7);
            //            parameters[0] = para_Pose[i];
            //            parameters[1] = para_Pose[j];
            //            parameters[2] = para_Ex_Pose_wheel[0];
            //            parameters[3] = para_Ix_sx_wheel[0];
            //            parameters[4] = para_Ix_sy_wheel[0];
            //            parameters[5] = para_Ix_sw_wheel[0];
            //            parameters[6] = para_Td_wheel[0];
            //            wheel_factor->check(const_cast<double **>(parameters.data()));
        }
    }

    if (USE_PLANE)
    {
        for (int i = 0; i < frame_count; i++)
        {
            PlaneFactor *plane_factor = new PlaneFactor();
            problem.AddResidualBlock(plane_factor, NULL, para_Pose[i], para_Ex_Pose_wheel[0], para_plane_R[0], para_plane_Z[0]);

            //            std::vector<const double *> parameters(4);
            //            parameters[0] = para_Pose[i];
            //            parameters[1] = para_Ex_Pose_wheel[0];
            //            parameters[2] = para_plane_R[0];
            //            parameters[3] = para_plane_Z[0];
            //            plane_factor->check(const_cast<double **>(parameters.data()));
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                if (it_per_id.estimate_flag == 1)
                    problem.SetParameterBlockConstant(para_Feature[feature_index]); // new
            }

            f_m_cnt++;
        }
    }

    /////////////////////////////////////
    // Line feature
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        // cout<<"linefeature perframe size:"<<it_per_id.used_num<<endl;
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        ++linefeature_index;

        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock(para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line); // p,q

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            Vector4d obs = it_per_frame.lineobs;
            lineProjectionFactor *f = new lineProjectionFactor(obs);
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[linefeature_index]);
            line_m_cnt++;
        }
    }
    ROS_INFO("lineFactor: %d, pointFactor:%d", line_m_cnt, f_m_cnt);

    // if(line_m_cnt > 20)
    // {
    //     double scale = std::min(f_m_cnt /(2. * line_m_cnt), 10.);
    //     lineProjectionFactor::sqrt_info =  scale * FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    //     std::cout << "========== Line factor weight ========== \n" << scale  << std::endl;
    // }

    ////////////////////////////////////////

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());

    double2vectorline();
    // printf("frame_count: %d \n", frame_count);

    if (frame_count < WINDOW_SIZE)
        return;

    // line
    double2vector2(); // Line pose change
    TicToc t_culling;
    f_manager.removeLineOutlier(Ps, tic, ric); // remove Line outlier
    // ROS_INFO("culling line feautre: %f ms", t_culling.toc());

    // line above

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2doubleline();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) // && is_imu_excited) //(solver_flag == NON_LINEAR ||is_imu_excited)
        {
            if (pre_integrations_wheel[1]->sum_dt < 10.0 && !(wdetect && wheelanomaly))
            {
                WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(wheel_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_Pose[1], para_Ex_Pose_wheel[0], para_Ix_sx_wheel[0], para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]},
                                                                               vector<int>{0});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        if (USE_PLANE)
        {
            PlaneFactor *plane_factor = new PlaneFactor();
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(plane_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_Ex_Pose_wheel[0], para_plane_R[0], para_plane_Z[0]},
                                                                           vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        int linefeature_index = -1;
        for (auto &it_per_id : f_manager.linefeature)
        {
            it_per_id.used_num = it_per_id.linefeature_per_frame.size();
            if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
                continue;
            ++linefeature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0)
                continue;

            for (auto &it_per_frame : it_per_id.linefeature_per_frame)
            {
                imu_j++;

                std::vector<int> drop_set;
                if (imu_i == imu_j)
                {
                    //                        drop_set = vector<int>{0, 2};   // marg pose and feature,  !!!! do not need marg, just drop they  !!!
                    continue;
                }
                else
                {
                    drop_set = vector<int>{2}; // marg feature
                }

                Vector4d obs = it_per_frame.lineobs;
                lineProjectionFactor *f = new lineProjectionFactor(obs);

                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                               vector<double *>{para_Pose[imu_j], para_Ex_Pose[0], para_LineFeature[linefeature_index]},
                                                                               drop_set);
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // line above

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
        addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])] = para_Ix_sx_wheel[0];
        addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])] = para_Ix_sy_wheel[0];
        addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])] = para_Ix_sw_wheel[0];

        addr_shift[reinterpret_cast<long>(para_plane_R[0])] = para_plane_R[0];
        addr_shift[reinterpret_cast<long>(para_plane_Z[0])] = para_plane_Z[0];

        addr_shift[reinterpret_cast<long>(para_Td_wheel[0])] = para_Td_wheel[0];
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2doubleline();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])] = para_Ix_sx_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])] = para_Ix_sy_wheel[0];
            addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])] = para_Ix_sw_wheel[0];

            addr_shift[reinterpret_cast<long>(para_plane_R[0])] = para_plane_R[0];
            addr_shift[reinterpret_cast<long>(para_plane_Z[0])] = para_plane_Z[0];
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            addr_shift[reinterpret_cast<long>(para_Td_wheel[0])] = para_Td_wheel[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }

    wheelanomaly = false;

    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    // printf("whole time for ceres: %f \n", t_whole.toc());
}
