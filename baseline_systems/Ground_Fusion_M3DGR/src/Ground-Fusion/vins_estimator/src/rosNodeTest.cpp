/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

// bounding box
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include <condition_variable>
Estimator estimator; //全局实例化

int syncflag = 0;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<std::vector<ObsPtr>> gnss_meas_buf;
queue<darknet_ros_msgs::BoundingBoxesConstPtr> boxes_buf;
std::mutex m_buf;

vector<Vector4d> vio_xyzt_buf;
vector<Vector4d> wheel_xyzt_buf;
vector<Vector3d> vio_xyz_buf;
vector<Vector3d> wheel_xyz_buf;

// gnss new
std::mutex m_time;
double next_pulse_time;
bool next_pulse_time_valid;
double time_diff_gnss_local;
bool time_diff_valid;
double latest_gnss_time;
double tmp_last_feature_time;
uint64_t feature_msg_counter;
int skip_parameter;
std::condition_variable con;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

void box_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &boxes_msg)
{
    m_buf.lock();
    boxes_buf.push(boxes_msg);

    m_buf.unlock();
}
void wheel_callback(const nav_msgs::OdometryConstPtr &odom_msg) // new
{

    // vehicle
    m_wheel_odom.lock();
    wheel_odom_buf.push(odom_msg);
    m_wheel_odom.unlock();

    double t = odom_msg->header.stamp.toSec();
    double rz_f = 0, rz_b = 0, t_f = 0, t_b = 0;
    double final_z = 0;

    while (!imu_buf.empty())
    {
        int Bg_z = 0.000000001;
        // sensor_msgs::ImuConstPtr imu_msg=imu_buf.front();

        double t_imu = imu_buf.front()->header.stamp.toSec();
        // cout<<"t - timu"<<t-t_imu<<endl;

        if (t - t_imu < 0.006 && t - t_imu > 0)
        {
            // cout << "success" << endl;
            rz_f = -imu_buf.front()->angular_velocity.y;
            // cout<<"rz:"<<rz2<<endl;
            t_f = t_imu;
            imu_buf.pop();

            if (!imu_buf.empty())
            {
                rz_b = -imu_buf.front()->angular_velocity.y;
                t_imu = imu_buf.front()->header.stamp.toSec();
                t_b = t_imu;
                imu_buf.pop();

                // final_z = (t - t_f) / (t_b - t_f) * rz_b + (t_b - t) / (t_b - t_f) * rz_f;
                final_z = rz_f + (rz_b - rz_f) / (t_b - t_f) * (t - t_f) - Bg_z;
                // cout << "final z:" << final_z << endl;
            }

            break;
        }
        else
        {
            // cout << "not" << endl;
            imu_buf.pop();
        }
    }
    double dx = odom_msg->twist.twist.linear.x;
    double dy = odom_msg->twist.twist.linear.y;
    double dz = odom_msg->twist.twist.linear.z;
    double rx = odom_msg->twist.twist.angular.x;
    double ry = odom_msg->twist.twist.angular.y;
    double rz = odom_msg->twist.twist.angular.z;

    Vector3d vel(dx, dy, dz);

    if (final_z != 0 && w_replace)
    {
        // cout<<"imu angular v:"<<rz_f<<endl;
        // cout<<"wheel angular v:"<<rz<<endl;
        // cout<<"angular vdiff:"<<rz_f-rz<<endl;
        Vector3d gyr(rx, ry, final_z);
        // Vector3d gyr(0,0,0);
        // cout << "ok" << endl;
        estimator.inputWheel(t, vel, gyr);
    }
    else
    {
        // cout<<"not ok"<<endl;
        Vector3d gyr(rx, ry, rz);
        estimator.inputWheel(t, vel, gyr);
        // cout<<"delta z:"<<final_z-rz<<endl;
        // cout<<"final z:"<<final_z<<endl;
        // cout<<"r z:"<<rz<<endl;
    }

    // for visualization
    Eigen::Vector3d rawP;
    Eigen::Quaterniond rawQ;
    double rawx = odom_msg->pose.pose.position.x;
    double rawy = odom_msg->pose.pose.position.y;
    double rawz = odom_msg->pose.pose.position.z;

    double rawqx = odom_msg->pose.pose.orientation.x;
    double rawqy = odom_msg->pose.pose.orientation.y;
    double rawqz = odom_msg->pose.pose.orientation.z;
    double rawqw = odom_msg->pose.pose.orientation.w;
    rawP[0] = rawx;
    rawP[1] = rawy;
    rawP[2] = rawz;
    rawQ.x() = rawqx;
    rawQ.y() = rawqy;
    rawQ.z() = rawqz;
    rawQ.w() = rawqw;
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    pubWheelRawodom(rawP, rawQ, header);

    Vector3d wheel_xyz;
    wheel_xyz[0] = rawx;
    wheel_xyz[1] = rawy;
    wheel_xyz[2] = 0;

    estimator.inputrawodom(t, wheel_xyz);

    return;
}

// gnss new
void gnss_ephem_callback(const GnssEphemMsgConstPtr &ephem_msg)
{
    EphemPtr ephem = msg2ephem(ephem_msg);
    estimator.inputEphem(ephem);
}

void gnss_glo_ephem_callback(const GnssGloEphemMsgConstPtr &glo_ephem_msg)
{
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    estimator.inputEphem(glo_ephem);
}

void gnss_iono_params_callback(const StampedFloat64ArrayConstPtr &iono_msg)
{
    double ts = iono_msg->header.stamp.toSec();
    std::vector<double> iono_params;
    std::copy(iono_msg->data.begin(), iono_msg->data.end(), std::back_inserter(iono_params));
    assert(iono_params.size() == 8);
    estimator.inputIonoParams(ts, iono_params);
}

void gnss_meas_callback(const GnssMeasMsgConstPtr &meas_msg)
{
    // cout<<"gnss meas"<<endl;
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);

    latest_gnss_time = time2sec(gnss_meas[0]->time);

    // cerr << "gnss ts is " << std::setprecision(20) << time2sec(gnss_meas[0]->time) << endl;
    if (!time_diff_valid)
    {
        cout << "time diff not valid:" << endl;
        return;
    }

    m_buf.lock();
    // cout<<"get gnss"<<endl;
    // cout<<"input gnss empty??"<<gnss_meas.empty()<<endl;

    // cout<<"inputgnss latest gnss ttime"<<latest_gnss_time<<endl<<endl<<endl;
    estimator.inputGNSS(latest_gnss_time, std::move(gnss_meas));
    gnss_meas_buf.push(std::move(gnss_meas));
    m_buf.unlock();
    con.notify_one();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    if (EQUALIZE) // new
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->apply(ptr->image, ptr->image);
        // ROS_INFO("Equalization Using!");
    }

    cv::Mat img = ptr->image.clone();
    return img;
}

cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    // depth has encoding TYPE_16UC1
    cv_bridge::CvImageConstPtr depth_ptr;
    // if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = sensor_msgs::image_encodings::MONO16;
        depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    }
    // else
    // depth_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO16);
    cv::Mat img = depth_ptr->image.clone();
    return img;
}

//合并相同时间戳的两个图像
// extract images with same timestamp from two topics
void sync_process()
{
    while (1)
    {

        int matchflag = 0;   // 是否获得成功的box匹配
        if (STEREO || DEPTH) // this is the case
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            darknet_ros_msgs::BoundingBoxesConstPtr backboxes, boxes;
            m_buf.lock();

            if (USE_YOLO)
            {

                if (!img0_buf.empty() && !img1_buf.empty())
                {
                    double time0 = img0_buf.front()->header.stamp.toSec();
                    double time1 = img1_buf.front()->header.stamp.toSec();
                    if (time0 < time1 - 0.003)
                    {
                        img0_buf.pop();
                        printf("throw img0\n");
                    }
                    else if (time0 > time1 + 0.003)
                    {
                        img1_buf.pop();
                        printf("throw img1\n");
                    }
                    else // img0 and img1 match
                    {
                        time = img0_buf.front()->header.stamp.toSec();
                        header = img0_buf.front()->header;
                        image0 = getImageFromMsg(img0_buf.front());
                        image1 = getDepthImageFromMsg(img1_buf.front());
                        // img0_buf.pop();

                        double backboxtime = 0, boxtime = 0;
                        // cout<<"boxes buf size:"<<boxes_buf.size()<<endl;
                        if (!boxes_buf.empty())
                        {
                            backboxes = boxes_buf.back();
                            backboxtime = backboxes->image_header.stamp.toSec();
                            if (backboxtime < time)
                            { // cout<<"<"<<endl<<endl;
                                // continue;//if会卡死
                                // sleep(0.1);  //实测，用while等待，buf不会更新
                                backboxes = boxes_buf.back();
                                backboxtime = backboxes->image_header.stamp.toSec();
                                // cout << setprecision(19) << "backbox time:" << backboxtime << " time:" << time << endl;
                            }
                        }
                        if ((backboxtime < time) && syncflag == 1)
                        {
                            syncflag = 0;
                            // cout<<"restart sync"<<endl;
                        }
                        if ((backboxtime > time || backboxtime == time) && syncflag == 0)
                        {
                            syncflag = 1;
                            // cout<<"success!"<<endl;//已经成功输出！
                            // cout << setprecision(19) << "backbox time:" << backboxtime << " time:" << time << endl;
                            // break;
                        }
                        if (syncflag && !boxes_buf.empty())
                        {
                            boxes = boxes_buf.front();
                            boxtime = boxes->image_header.stamp.toSec();
                            while (boxtime < time && !boxes_buf.empty())
                            {
                                boxes = boxes_buf.front();
                                boxtime = boxes->image_header.stamp.toSec();
                                boxes_buf.pop();
                            }
                            if (boxtime == time)
                            {
                                // cout<<"666"<<endl;
                                // cout << setprecision(19) << "backbox time:" << boxtime << " time:" << time << endl;
                                matchflag = 1;
                                // break;
                            }
                        }

                        if (syncflag)
                        {
                            img0_buf.pop();
                            img1_buf.pop();
                        }

                        // printf("find img0 and img1\n");
                    }
                }
            }
            else // org
            {

                if (!img0_buf.empty() && !img1_buf.empty())
                {
                    double time0 = img0_buf.front()->header.stamp.toSec();
                    double time1 = img1_buf.front()->header.stamp.toSec();
                    if (time0 < time1 - 0.003)
                    {
                        img0_buf.pop();
                        printf("throw img0\n");
                    }
                    else if (time0 > time1 + 0.003)
                    {
                        img1_buf.pop();
                        printf("throw img1\n");
                    }
                    else
                    {
                        time = img0_buf.front()->header.stamp.toSec();
                        header = img0_buf.front()->header;
                        image0 = getImageFromMsg(img0_buf.front());
                        img0_buf.pop();

                        if (DEPTH)
                        {
                            image1 = getDepthImageFromMsg(img1_buf.front());
                            img1_buf.pop();
                        }
                        else
                        {
                            image1 = getImageFromMsg(img1_buf.front());
                            img1_buf.pop();
                        }
                        // printf("find img0 and img1\n");
                    }
                }
            }

            m_buf.unlock();

            // if (!image0.empty())
            if (!image1.empty()) // this must happen
            {
                if (USE_LINE)
                    estimator.inputImagewithline(time, image0, image1);
                else
                {
                    if (USE_YOLO && syncflag && matchflag)
                    {
                        estimator.inputImagebox(time, boxes, image0, image1); // here
                    }
                    else
                    {
                        estimator.inputImage(time, image0, image1);
                    }
                }
            }
        }
        else // mono case
        {
            cv::Mat image;
            darknet_ros_msgs::BoundingBoxesConstPtr backboxes, boxes;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());

                if (USE_YOLO)
                {
                    double backboxtime = 0, boxtime = 0;
                    // cout<<"boxes buf size:"<<boxes_buf.size()<<endl;
                    if (!boxes_buf.empty())
                    {
                        backboxes = boxes_buf.back();
                        backboxtime = backboxes->image_header.stamp.toSec();
                        if (backboxtime < time)
                        { // cout<<"<"<<endl<<endl;
                            // continue;//if会卡死
                            // sleep(0.1);  //实测，用while等待，buf不会更新
                            backboxes = boxes_buf.back();
                            backboxtime = backboxes->image_header.stamp.toSec();
                            // cout << setprecision(19) << "backbox time:" << backboxtime << " time:" << time << endl;
                        }
                    }
                    if ((backboxtime < time) && syncflag == 1)
                    {
                        syncflag = 0;
                        // cout<<"restart sync"<<endl;
                    }
                    if ((backboxtime > time || backboxtime == time) && syncflag == 0)
                    {
                        syncflag = 1;

                        // cout << setprecision(19) << "backbox time:" << backboxtime << " time:" << time << endl;
                        // break;
                    }
                    if (syncflag && !boxes_buf.empty())
                    {
                        boxes = boxes_buf.front();
                        boxtime = boxes->image_header.stamp.toSec();
                        while (boxtime < time && !boxes_buf.empty())
                        {
                            boxes = boxes_buf.front();
                            boxtime = boxes->image_header.stamp.toSec();
                            boxes_buf.pop();
                        }
                        if (boxtime == time)
                        {

                            matchflag = 1;
                        }
                    }

                } // use yolo

                if (USE_YOLO == 1)
                {
                    if (syncflag)
                    {
                        img0_buf.pop();
                    }
                }
                else if (USE_YOLO == 0)
                {
                    img0_buf.pop();
                }
            }

            m_buf.unlock();
            if (USE_YOLO)
            {
                if (!image.empty() && syncflag) // 已经成功对齐时间戳
                {

                    if (USE_LINE)
                    {
                        estimator.inputImagewithline(time, image);
                    }

                    else
                    {
                        if (matchflag == 0)
                        {
                            estimator.inputImage(time, image);
                        }

                        else if (matchflag == 1)
                        {
                            estimator.inputImagebox(time, boxes, image);
                        }
                    }
                }
            }
            else if (USE_YOLO == 0)
            {
                if (!image.empty())
                {

                    if (USE_LINE)
                    {
                        estimator.inputImagewithline(time, image);
                    }

                    else
                    {

                        estimator.inputImage(time, image);
                    }
                }
            }
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // vehicle
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();

    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while (!feature_buf.empty())
            feature_buf.pop();
        while (!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void local_trigger_info_callback(const GnssTimePulseInfoMsgConstPtr &trigger_msg)
{
    std::lock_guard<std::mutex> lg(m_time);
}

void gnss_tp_info_callback(const GnssTimePulseInfoMsgConstPtr &tp_msg)
{
    gtime_t tp_time = gpst2time(tp_msg->time.week, tp_msg->time.tow);
    if (tp_msg->utc_based || tp_msg->time_sys == SYS_GLO)
        tp_time = utc2gpst(tp_time);
    else if (tp_msg->time_sys == SYS_GAL)
        tp_time = gst2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_BDS)
        tp_time = bdt2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_NONE)
    {
        std::cerr << "Unknown time system in GNSSTimePulseInfoMsg.\n";
        return;
    }
    double gnss_ts = time2sec(tp_time);

    std::lock_guard<std::mutex> lg(m_time);
    next_pulse_time = gnss_ts;
    next_pulse_time_valid = true;
}

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if (argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    /*
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cout << "config_file: "<< config_file <<endl;
*/
    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 5000, imu_callback, ros::TransportHints().tcpNoDelay()); // 2000
    ros::Subscriber sub_wheel = n.subscribe(WHEEL_TOPIC, 5000, wheel_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    ros::Subscriber sub_box = n.subscribe("/darknet_ros/bounding_boxes", 100, box_callback);
    // sleep(0.2);//for time sync

    // gnss new

    ros::Subscriber sub_ephem, sub_glo_ephem, sub_gnss_meas, sub_gnss_iono_params;
    ros::Subscriber sub_gnss_time_pluse_info, sub_local_trigger_info;
    if (GNSS_ENABLE)
    {
        sub_ephem = n.subscribe(GNSS_EPHEM_TOPIC, 100, gnss_ephem_callback);
        sub_glo_ephem = n.subscribe(GNSS_GLO_EPHEM_TOPIC, 100, gnss_glo_ephem_callback);
        sub_gnss_meas = n.subscribe(GNSS_MEAS_TOPIC, 100, gnss_meas_callback);
        sub_gnss_iono_params = n.subscribe(GNSS_IONO_PARAMS_TOPIC, 100, gnss_iono_params_callback);

        if (GNSS_LOCAL_ONLINE_SYNC)
        {
            sub_gnss_time_pluse_info = n.subscribe(GNSS_TP_INFO_TOPIC, 100,
                                                   gnss_tp_info_callback);
            sub_local_trigger_info = n.subscribe(LOCAL_TRIGGER_INFO_TOPIC, 100,
                                                 local_trigger_info_callback);
            time_diff_gnss_local = GNSS_LOCAL_TIME_DIFF;
        }
        else
        {
            time_diff_gnss_local = GNSS_LOCAL_TIME_DIFF;
            estimator.inputGNSSTimeDiff(time_diff_gnss_local);
            time_diff_valid = true;
        }
    }

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
