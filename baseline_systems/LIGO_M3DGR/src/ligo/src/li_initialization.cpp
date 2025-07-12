/*
 * BSD 3-Clause License

 *  Copyright (c) 2025, Dongjiao He
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "li_initialization.h"
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::Imu imu_last, imu_next;
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());
double s_plot[MAXN], s_plot3[MAXN];

bool first_gps = false;
Eigen::Vector3d first_gps_lla;
Eigen::Vector3d first_gps_ecef;
condition_variable sig_buffer;
int loop_count = 0;
int scan_count_point = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<double>               time_buffer;
std::deque<sensor_msgs::Imu::Ptr> imu_deque;
std::queue<std::vector<ObsPtr>> gnss_meas_buf;
std::queue<nav_msgs::OdometryPtr> nmea_meas_buf;

void gnss_ephem_callback(const GnssEphemMsgConstPtr &ephem_msg)
{
    EphemPtr ephem = msg2ephem(ephem_msg);
    p_gnss->p_assign->inputEphem(ephem);
}

void gnss_glo_ephem_callback(const GnssGloEphemMsgConstPtr &glo_ephem_msg)
{
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    p_gnss->p_assign->inputEphem(glo_ephem);
}

void gnss_iono_params_callback(const StampedFloat64ArrayConstPtr &iono_msg)
{
    double ts = iono_msg->header.stamp.toSec();
    std::vector<double> iono_params;
    std::copy(iono_msg->data.begin(), iono_msg->data.end(), std::back_inserter(iono_params));
    assert(iono_params.size() == 8);
    p_gnss->inputIonoParams(ts, iono_params);
}

void rtk_pvt_callback(const GnssPVTSolnMsgConstPtr &groundt_pvt)
{
    double ts = time2sec(gst2time(groundt_pvt->time.week, groundt_pvt->time.tow));
    p_gnss->inputpvt(ts, groundt_pvt->latitude, groundt_pvt->longitude, groundt_pvt->altitude, groundt_pvt->carr_soln, groundt_pvt->diff_soln);
}

void rtk_lla_callback(const sensor_msgs::NavSatFixConstPtr &lla_msg)
{
    double ts = lla_msg->header.stamp.toSec();
    p_gnss->inputlla(ts, lla_msg->latitude, lla_msg->longitude, lla_msg->altitude);
}

void gnss_meas_callback(const GnssMeasMsgConstPtr &meas_msg)
{
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);
    
    latest_gnss_time = time2sec(gnss_meas[0]->time);
    // printf("gnss time: %f\n", latest_gnss_time);

    // cerr << "gnss ts is " << std::setprecision(20) << time2sec(gnss_meas[0]->time) << endl;
    if (!time_diff_valid)   return;

    // mtx_buffer.lock();
    gnss_meas_buf.push(std::move(gnss_meas)); // ?
    // mtx_buffer.unlock();
    // sig_buffer.notify_all(); // notify_one()?
}

void gnss_meas_callback_urbannav(const nlosExclusion::GNSS_Raw_ArrayConstPtr &meas_msg)
{
    rtklib_gnss_meas_callback(meas_msg, gnss_meas_buf);
}

void nmea_meas_callback(const nav_msgs::OdometryConstPtr &meas_msg)
{    
    nav_msgs::OdometryPtr nmea_meas(new nav_msgs::Odometry(*meas_msg));
    last_nmea_time = nmea_meas->header.stamp.toSec();
    nmea_meas_buf.push(std::move(nmea_meas)); // ?
}

void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
{
    if (gpsMsg->status.status != 0)
        return;

    Eigen::Vector3d trans_local_;
    if (!first_gps) {
        first_gps = true;
        Eigen::Vector3d geo;
        geo << gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude;
        first_gps_lla = geo;
        first_gps_ecef = geo2ecef(geo);
    }
    Eigen::Vector3d cur_ecef = geo2ecef(Eigen::Vector3d(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude));
    trans_local_ = ecef2enu(first_gps_lla, cur_ecef - first_gps_ecef);

    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = gpsMsg->header.stamp;
    // gps_odom->header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    // gps_odom->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    // pubGpsOdom.publish(gps_odom);
    // gpsQueue.push_back(gps_odom);
    nmea_meas_buf.push(nav_msgs::OdometryPtr(new nav_msgs::Odometry(gps_odom)));
}

void local_trigger_info_callback(const ligo::LocalSensorExternalTriggerConstPtr &trigger_msg) // pps time sync
{
    std::lock_guard<std::mutex> lg(m_time);

    if (next_pulse_time_valid)
    {
        time_diff_gnss_local = next_pulse_time - trigger_msg->header.stamp.toSec();
        p_gnss->inputGNSSTimeDiff(time_diff_gnss_local);
        if (!time_diff_valid)       // just get calibrated
            std::cout << "time difference between GNSS and LI-Sensor got calibrated: "
                << std::setprecision(15) << time_diff_gnss_local << " s\n";
        time_diff_valid = true;
    }
}

void gnss_tp_info_callback(const GnssTimePulseInfoMsgConstPtr &tp_msg) // time stamp of GNSS signal
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

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    // mtx_buffer.lock();
    scan_count ++;
    // double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        return;
    }

    last_timestamp_lidar = msg->header.stamp.toSec();

    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
    p_pre->process(msg, ptr);
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    { 
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    }
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    // mtx_buffer.lock();
    // double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();    

    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
    p_pre->process(msg, ptr); 
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    }
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    // mtx_buffer.lock();

    // publish_count ++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = msg->header.stamp.toSec();
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);
        
        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas, queue<std::vector<ObsPtr>> &gnss_msg, queue<nav_msgs::OdometryPtr> &nmea_msg)
{
    if (nolidar)
    {
        if (is_first_gnss && !NMEA_ENABLE)
        {
            if (gnss_meas_buf.empty())
            {
                // imu_pushed = false;
                return false;
            }
            else
            {
                // double imu_time = imu_deque.front()->header.stamp.toSec();
                // double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                // if (last_timestamp_imu < front_gnss_ts - time_diff_gnss_local)
                // {
                    // return false;
                // }
                // while (front_gnss_ts - imu_time > time_diff_gnss_local) // wrong
                // {
                    // imu_last = *(imu_deque.front());
                    // imu_deque.pop_front();
                    // if(imu_deque.empty()) break;
                    // imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    // imu_next = *(imu_deque.front());
                // }
                // else
                while (!imu_deque.empty())
                {
                    imu_deque.pop_front();
                }
                {
                    is_first_gnss = false;
                }
            }
        }

        if (is_first_nmea && NMEA_ENABLE)
        {
            if (nmea_meas_buf.empty())
            {
                return false;
            }
            else
            {
                while (!imu_deque.empty())
                {
                    imu_deque.pop_front();
                }
                {
                    is_first_nmea = false;
                }
            }
        }


        if (imu_deque.empty())
        {
            return false;
        }
        
        imu_first_time = imu_deque.front()->header.stamp.toSec(); // 

        if ((latest_gnss_time < time_diff_gnss_local + imu_first_time + lidar_time_inte) && GNSS_ENABLE)
        {
            return false;
        }

        if ((last_nmea_time < time_diff_nmea_local + imu_first_time + lidar_time_inte) && NMEA_ENABLE)
        {
            return false;
        }

        if (last_timestamp_imu < imu_first_time + lidar_time_inte)
        {
            return false;
        }

        if (imu_deque.empty())
        {
            cout << "could not be here" << endl;
            return false;
        }

        if (!imu_pushed)
        { 
            double imu_time = imu_deque.front()->header.stamp.toSec();
            // imu_first_time = imu_time;

            double imu_last_time = imu_deque.back()->header.stamp.toSec();
            if (imu_last_time - imu_first_time < lidar_time_inte)
            {
                return false;
            }
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_)
            {
                imu_next = *(imu_deque.front());
                meas.imu.shrink_to_fit();
                while (imu_time - imu_first_time < lidar_time_inte)
                {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if(imu_deque.empty()) break;
                    imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    imu_next = *(imu_deque.front());
                }
                if (!gnss_meas_buf.empty())
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    while (front_gnss_ts < imu_first_time + lidar_time_inte + time_diff_gnss_local)
                    {
                        gnss_meas_buf.pop();
                        if(gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    }
                    if (meas.imu.empty())
                    {
                        return false;
                    }
                }
                if (!nmea_meas_buf.empty())
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    while (front_nmea_ts < imu_first_time + lidar_time_inte + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    }
                    if (meas.imu.empty())
                    {
                        return false;
                    }
                }
            }
            imu_pushed = true;
        }

        if (GNSS_ENABLE)
        {
            if (!gnss_meas_buf.empty()) // or can wait for a short time?
            {
                // double back_gnss_ts = time2sec(gnss_meas_buf.back()[0]->time);
                
                // if (back_gnss_ts - imu_first_time < time_diff_gnss_local + lidar_time_inte)
                // {
                //     return false;
                // }
                double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                while (front_gnss_ts - imu_first_time < time_diff_gnss_local + lidar_time_inte) 
                {
                    gnss_msg.push(gnss_meas_buf.front());
                    gnss_meas_buf.pop();
                    if (gnss_meas_buf.empty()) break;
                    front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                }

                if (!gnss_msg.empty())
                {
                    imu_pushed = false;
                    return true;
                }
            }

            // if (gnss_meas_buf.empty())
            // {
            //     wait_num ++;
            //     if (wait_num > 2) 
            //     {
            //         wait_num = 0;
            //     }
            //     else
            //     {
            //         return false;
            //     }
            // }
        }
        if (NMEA_ENABLE)
        {
            if (!nmea_meas_buf.empty()) // or can wait for a short time?
            {
                double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                while (front_nmea_ts - imu_first_time < time_diff_nmea_local + lidar_time_inte) 
                {
                    nmea_msg.push(nmea_meas_buf.front());
                    nmea_meas_buf.pop();
                    if (nmea_meas_buf.empty()) break;
                    front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                }

                if (!nmea_msg.empty())
                {
                    imu_pushed = false;
                    return true;
                }
            }
        }        
        imu_pushed = false;
        return true;
    }
    else
    {
    if (!imu_en)
    {
        if (is_first_gnss && GNSS_ENABLE)
        {
            if (gnss_meas_buf.empty())
            {
                return false;
            }
            else
            {
                while (!lidar_buffer.empty())
                {
                    lidar_buffer.pop_front();
                }
            }
        }
        if (!lidar_buffer.empty())
        {
            if (!lidar_pushed)
            {
                meas.lidar = lidar_buffer.front();
                meas.lidar_beg_time = time_buffer.front();
                lose_lid = false;
                if(meas.lidar->points.size() < 1) 
                {
                    cout << "lose lidar" << std::endl;
                    // return false;
                    lose_lid = true;
                }
                else
                {
                    double end_time = meas.lidar->points.back().curvature;
                    for (auto pt: meas.lidar->points)
                    {
                        if (pt.curvature > end_time)
                        {
                            end_time = pt.curvature;
                        }
                    }
                    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                    meas.lidar_last_time = lidar_end_time;
                }
                lidar_pushed = true;
            }
            
            if (GNSS_ENABLE)
            {
                if (!gnss_meas_buf.empty()) // or can wait for a short time?
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    while (front_gnss_ts < meas.lidar_beg_time + time_diff_gnss_local) // 0.05
                    {
                        ROS_WARN("throw gnss, only should happen at the beginning 542");
                        gnss_meas_buf.pop();
                        if (gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    }
                    if (!gnss_meas_buf.empty())
                    {
                    while ((!lose_lid && (front_gnss_ts <= lidar_end_time + time_diff_gnss_local)) || (lose_lid && (front_gnss_ts <= meas.lidar_beg_time + time_diff_gnss_local + lidar_time_inte) ))
                    {
                        gnss_msg.push(gnss_meas_buf.front());
                        gnss_meas_buf.pop();
                        if (gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    }
                    if (!gnss_msg.empty())
                    {
                        time_buffer.pop_front();
                        lidar_buffer.pop_front();
                        lidar_pushed = false;
                        return true;
                    }
                    }
                }

                // if (gnss_meas_buf.empty())
                // {
                //     wait_num ++;
                //     if (wait_num > 2) 
                //     {
                //         wait_num = 0;
                //     }
                //     else
                //     {
                //         return false;
                //     }
                // }
            }
            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty()) // or can wait for a short time?
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                    while (front_nmea_ts < meas.lidar_beg_time + time_diff_nmea_local) // 0.05
                    {
                        ROS_WARN("throw nmea, only should happen at the beginning 542");
                        nmea_meas_buf.pop();
                        if (nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                    }
                    if (!nmea_meas_buf.empty())
                    {
                    while ((!lose_lid && (front_nmea_ts <= lidar_end_time + time_diff_nmea_local)) || (lose_lid && (front_nmea_ts <= meas.lidar_beg_time + time_diff_nmea_local + lidar_time_inte) ))
                    {
                        nmea_msg.push(nmea_meas_buf.front());
                        nmea_meas_buf.pop();
                        if (nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                    }
                    if (!nmea_msg.empty())
                    {
                        time_buffer.pop_front();
                        lidar_buffer.pop_front();
                        lidar_pushed = false;
                        return true;
                    }
                    }
                }
            }
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_pushed = false;
            if (!lose_lid)
            {
                return true;
            }
            else
            {
                return false;
            }
        }        
        return false;
    }

    if (0) // (is_first_gnss && GNSS_ENABLE)
    {
        if (gnss_meas_buf.empty())
        {
            return false;
        }
        else
        {
            while (!lidar_buffer.empty())
            {
                lidar_buffer.pop_front();
            }
            is_first_gnss = false;
            while (!imu_deque.empty())
            {
                imu_deque.pop_front();
            }
        }
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        lose_lid = false;
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if(meas.lidar->points.size() < 1) 
        {
            cout << "lose lidar" << endl;
            lose_lid = true;
            // lidar_buffer.pop_front();
            // time_buffer.pop_front();
            // return false;
        }
        else
        {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            // cout << "check time lidar:" << end_time << endl;
            meas.lidar_last_time = lidar_end_time;
        }
        lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time + 2))
    {
        // lidar_pushed = false;
        return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte + 2)
    {
        // lidar_pushed = false;
        return false;
    }

    if (!lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            imu_next = *(imu_deque.front());
            meas.imu.shrink_to_fit();
            while (imu_time < lidar_end_time)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                imu_next = *(imu_deque.front());
            }
            if (GNSS_ENABLE)
            {
                if (!gnss_meas_buf.empty())
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take timedouble front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    while (front_gnss_ts < lidar_end_time + time_diff_gnss_local)
                    {
                        gnss_meas_buf.pop();
                        if(gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    }
                }
            }
            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty())
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); 
                    while (front_nmea_ts < lidar_end_time + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    }
                }
            }
        }
        imu_pushed = true;
    }
    
    if (lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            meas.imu.shrink_to_fit();

            imu_next = *(imu_deque.front());
            while (imu_time < meas.lidar_beg_time + lidar_time_inte)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                imu_next = *(imu_deque.front());
            }

            if (GNSS_ENABLE)
            {
                if (!gnss_meas_buf.empty())
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    while (front_gnss_ts < meas.lidar_beg_time + lidar_time_inte + time_diff_gnss_local)
                    {
                        gnss_meas_buf.pop();
                        if(gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    }
                }
            }

            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty())
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    while (front_nmea_ts < meas.lidar_beg_time + lidar_time_inte + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    }
                }
            }
        }

        imu_pushed = true;
    }

    if (GNSS_ENABLE)
    {
        if (!gnss_meas_buf.empty()) // or can wait for a short time?
        {
            double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
            while ((!lose_lid && (front_gnss_ts < lidar_end_time + time_diff_gnss_local)) || (lose_lid && (front_gnss_ts < meas.lidar_beg_time + time_diff_gnss_local + lidar_time_inte) )) // (front_gnss_ts >= meas.lidar_beg_time + time_diff_gnss_local) && 
            {
                gnss_msg.push(gnss_meas_buf.front());
                gnss_meas_buf.pop();
                if (gnss_meas_buf.empty()) break;
                front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
            }
            if (!gnss_msg.empty())
            {
                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                imu_pushed = false;
                return true;
            }
        }
    }
    if (NMEA_ENABLE)
    {
        if (!nmea_meas_buf.empty()) // or can wait for a short time?
        {
            double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
            while ((!lose_lid && (front_nmea_ts < lidar_end_time + time_diff_nmea_local)) || (lose_lid && (front_nmea_ts < meas.lidar_beg_time + time_diff_nmea_local + lidar_time_inte) )) 
            {
                nmea_msg.push(nmea_meas_buf.front());
                nmea_meas_buf.pop();
                if (nmea_meas_buf.empty()) break;
                front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
            }
            if (!nmea_msg.empty())
            {
                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                imu_pushed = false;
                return true;
            }
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
    }
}

