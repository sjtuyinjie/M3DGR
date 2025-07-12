#include "utility.h"
#include "livox_ros_driver/CustomMsg.h"
#include "lio_sam/cloud_info.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

struct LiovxPointCustomMsg
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    uint16_t ring;
    uint16_t tag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LiovxPointCustomMsg,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring)(uint16_t, tag, tag))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class CloudProcess : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    std::deque<livox_ros_driver::CustomMsg> cloudQueue_livox;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<LiovxPointCustomMsg>::Ptr tmpLivoxCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;

    int deskewFlag;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

public:
    CloudProcess() : deskewFlag(0)
    {
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &CloudProcess::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &CloudProcess::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        subLaserCloud = (sensor == SensorType::LIVOX)
                            ? nh.subscribe(pointCloudTopic, 5, &CloudProcess::cloudHandler2, this, ros::TransportHints().tcpNoDelay())
                            : nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &CloudProcess::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpLivoxCloudIn.reset(new pcl::PointCloud<LiovxPointCustomMsg>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        fullCloud->clear();

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~CloudProcess() {}

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg, sensor);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler2(const livox_ros_driver::CustomMsgConstPtr &laserCloudMsg)
    {
        if (!cachePointCloud2(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    void moveFromCustomMsg(livox_ros_driver::CustomMsg &Msg, pcl::PointCloud<PointXYZIRT> &cloud)
    {
        cloud.clear();
        cloud.reserve(Msg.point_num);
        PointXYZIRT point;

        cloud.header.frame_id = Msg.header.frame_id;
        cloud.header.stamp = Msg.header.stamp.toNSec() / 1000;
        cloud.header.seq = Msg.header.seq;

        for (uint i = 0; i < Msg.point_num - 1; i++)
        {
            point.x = Msg.points[i].x;
            point.y = Msg.points[i].y;
            point.z = Msg.points[i].z;
            point.intensity = Msg.points[i].reflectivity;
            // point.tag = Msg.points[i].tag;
            point.time = Msg.points[i].offset_time * 1e-9;
            point.ring = Msg.points[i].line;
            cloud.push_back(point);
        }
    }

    bool cachePointCloud2(const livox_ros_driver::CustomMsgConstPtr &laserCloudMsg)
    {
        cloudQueue_livox.push_back(*laserCloudMsg);
        if (cloudQueue_livox.size() <= 2)
            return false;
        // convert cloud
        livox_ros_driver::CustomMsg currentCloudMsg = std::move(cloudQueue_livox.front());
        cloudQueue_livox.pop_front();

        moveFromCustomMsg(currentCloudMsg, *laserCloudIn);
        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        return true;
    }
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::ROBOSENSE)
        {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++)
            {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        // imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // if (imuType == 0)
            {
                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            }

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;
        static float sync_diff_time = 0.1; // (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;
        int j;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            j = i;
            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        startOdomMsg = odomQueue[j - 1];
        std::cerr << "Invalid index j: " << j << " or odomQueue size: " << odomQueue.size() << std::endl;

        // std::cout << ROS_TIME(&startOdomMsg) - timeScanCur << " s" << std::endl;
        //  TODO: 插值获取对应时间的位姿
        double roll, pitch, yaw, x, y, z;
        interOdom(timeScanCur, x, y, z, roll, pitch, yaw);
        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        cloudInfo.odomAvailable = true;
    }

    void interOdom(double &realTime, double &x, double &y, double &z, double &roll, double &pitch, double &yaw)
    {
        // std::lock_guard<std::mutex> lock2(mtx_odom_msg_buf);
        nav_msgs::Odometry startOdomMsg, nextOdomMsg;
        int i, j;
        for (i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            j = i;

            if (ROS_TIME(&startOdomMsg) < realTime)
                continue;
            else
                break;
        }
        startOdomMsg = odomQueue[j - 1];
        nextOdomMsg = odomQueue[j];

        double s = (realTime - ROS_TIME(&startOdomMsg)) / (ROS_TIME(&nextOdomMsg) - ROS_TIME(&startOdomMsg));
        x = startOdomMsg.pose.pose.position.x + s * (nextOdomMsg.pose.pose.position.x - startOdomMsg.pose.pose.position.x);
        y = startOdomMsg.pose.pose.position.y + s * (nextOdomMsg.pose.pose.position.y - startOdomMsg.pose.pose.position.y);
        z = startOdomMsg.pose.pose.position.z + s * (nextOdomMsg.pose.pose.position.z - startOdomMsg.pose.pose.position.z);

        Eigen::Quaterniond q_pre(startOdomMsg.pose.pose.orientation.w, startOdomMsg.pose.pose.orientation.x,
                                 startOdomMsg.pose.pose.orientation.y, startOdomMsg.pose.pose.orientation.z);
        Eigen::Quaterniond q_next(nextOdomMsg.pose.pose.orientation.w, nextOdomMsg.pose.pose.orientation.x,
                                  nextOdomMsg.pose.pose.orientation.y, nextOdomMsg.pose.pose.orientation.z);

        Eigen::Quaterniond q_cur = q_pre.slerp(s, q_next);

        tf::Quaternion orientation(q_cur.x(), q_cur.y(), q_cur.z(), q_cur.w());

        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    }

    void interOdom(double &realTime, Eigen::Isometry3d &odom)
    {
        // std::lock_guard<std::mutex> lock2(mtx_odom_msg_buf);
        static double t_raw[3];
        static Eigen::Map<Eigen::Vector3d> t(t_raw);
        nav_msgs::Odometry startOdomMsg, nextOdomMsg;
        int i, j;
        for (i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            j = i;

            if (ROS_TIME(&startOdomMsg) < realTime)
                continue;
            else
                break;
        }
        startOdomMsg = odomQueue[j - 1];
        nextOdomMsg = odomQueue[j];

        double s = (realTime - ROS_TIME(&startOdomMsg)) / (ROS_TIME(&nextOdomMsg) - ROS_TIME(&startOdomMsg));
        t_raw[0] = startOdomMsg.pose.pose.position.x + s * (nextOdomMsg.pose.pose.position.x - startOdomMsg.pose.pose.position.x);
        t_raw[1] = startOdomMsg.pose.pose.position.y + s * (nextOdomMsg.pose.pose.position.y - startOdomMsg.pose.pose.position.y);
        t_raw[2] = startOdomMsg.pose.pose.position.z + s * (nextOdomMsg.pose.pose.position.z - startOdomMsg.pose.pose.position.z);
        // t_raw[0] = t_raw[1] = t_raw[2] = 0;

        Eigen::Quaterniond q_pre(startOdomMsg.pose.pose.orientation.w, startOdomMsg.pose.pose.orientation.x,
                                 startOdomMsg.pose.pose.orientation.y, startOdomMsg.pose.pose.orientation.z);
        Eigen::Quaterniond q_next(nextOdomMsg.pose.pose.orientation.w, nextOdomMsg.pose.pose.orientation.x,
                                  nextOdomMsg.pose.pose.orientation.y, nextOdomMsg.pose.pose.orientation.z);
        odom.setIdentity();
        odom.rotate(q_pre.slerp(s, q_next));
        odom.pretranslate(t);
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        Eigen::Isometry3d Pose0_inv, Pc;

        float verticalAngle;
        const float ang_res_y = 2.5;
        const float ang_bottom = 30.0+0.1;

        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(cloudInfo.initialGuessRoll, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(cloudInfo.initialGuessPitch, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(cloudInfo.initialGuessYaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond rotation = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
        odom(0, 3) = cloudInfo.initialGuessX;
        odom(1, 3) = cloudInfo.initialGuessY;
        odom(2, 3) = cloudInfo.initialGuessZ;
        odom.block(0, 0, 3, 3) = rotation.toRotationMatrix();

        Eigen::Isometry3d Pose0(odom);
        Pose0_inv = Pose0.inverse();
        Eigen::Vector3d oriP;

        // range image projection
        int point_filter_num = 1;
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            int rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
           
            //int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            if (i % point_filter_num != 0)
                continue;

            {
                double real = timeScanCur + laserCloudIn->points[i].time; //  time单点相对帧首的时间戳
                interOdom(real, Pc);

                Eigen::Vector3d newp = Pose0_inv * Pc * (oriP << thisPoint.x, thisPoint.y, thisPoint.z).finished();
                thisPoint.x = newp[0], thisPoint.y = newp[1], thisPoint.z = newp[2];
            }
            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(&pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    CloudProcess CP;

    ROS_INFO("\033[1;32m----> Cloud Process Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
