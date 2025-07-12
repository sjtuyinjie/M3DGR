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

#include "handler.h"

bool GTinLocal, LCinLocal, RTKinLocal = true; // visualization

double latest_gnss_time_u = 0;
double last_gnss_time = 0;
std::mutex m_buf;
GNSS_Tools m_GNSS_Tools;
Eigen::Vector3d m_pose_enu; // pose in enu from initialization
std::deque<nav_msgs::Odometry> GNSSQueue;
std::vector<Eigen::Vector3d> sat_pos;
std::vector<Eigen::Vector3d> sat_vel;

#ifdef process_ppp
    std::vector<Eigen::Vector4d> ppp_ecef;
    std::vector<Eigen::Matrix<double, 7, 1>> ppp_sol;
    Eigen::Vector3d first_lla_pvt, first_xyz_ecef_pvt;
    bool first_pvt = true;
#endif
/**
 * @brief variance estimation (sigma square)
 * @param nlosExclusion::GNSS_Raw_Array GNSS_data
 * @return weight_matrix
 @ 
*/
double eleSRNVarCal(double ele, double snr)
{
    int model = 0; // 0: ele&&SNR model. 1: RTKLIB ele model. 2: DLR Ele model
    Eigen::Matrix<double,4,1> parameters;
    parameters<<50.0, 30.0, 30.0, 10.0; // loosely coupled 
    // parameters<<50.0, 30.0, 20.0, 30.0; // loosely coupled 
    double snr_1 = parameters(0); // T = 50
    double snr_A = parameters(1); // A = 30
    double snr_a = parameters(2);// a = 30
    double snr_0 = parameters(3); // F = 10

    double snr_R = snr;
    // snr_R = 40;
    double elR = ele;
    double q_R_1 = 1 / (pow(( sin(elR * 3.1415926/180.0 )),2));
    double q_R_2 = pow(10,(-(snr_R - snr_1) / snr_a));
    double q_R_3 = (((snr_A / (pow(10,(-(snr_0 - snr_1) / snr_a))) - 1) / (snr_0 - snr_1)) * (snr_R - snr_1) + 1);
    double q_R = q_R_1* (q_R_2 * q_R_3);
    
    if(model == 0)
    {
        double weghting =(1.0/float(q_R)); // uncertainty: cofactor_[i] larger, larger uncertainty
        return (1.0/weghting);
    }
    return 0.0;
}

/* calculate the variance (sigma square) */
// bool calVar(ObsPtr &obs)
// {
//     double var = 0;

//     /* variance for r_i_sat */
//     obs->r_i_sat.var = eleSRNVarCal(obs->r_i_sat.elevation, obs->r_i_sat.snr);

//     /* variance for r_m_sat */
//     obs->r_m_sat.var = eleSRNVarCal(obs->r_m_sat.elevation, obs->r_m_sat.snr);

//     /* variance for u_m_sat */
//     obs->u_m_sat.var = eleSRNVarCal(obs->u_m_sat.elevation, obs->u_m_sat.snr);

//     /* variance for u_i_sat */
//     obs->var = eleSRNVarCal(obs->Uelevation, obs->Usnr);

//     return true;
// }

/* transform the gnss raw data to map format */
bool gnssRawArray2map(nlosExclusion::GNSS_Raw_Array gnss_data, std::map<int, nlosExclusion::GNSS_Raw> &epochGnssMap)
{
    for(int i = 0; i < gnss_data.GNSS_Raws.size(); i++)
    {
        int prn = int(gnss_data.GNSS_Raws[i].prn_satellites_index);
        epochGnssMap[prn] = gnss_data.GNSS_Raws[i];
    }
    return true;
}    
/* subscribe the odometry from RTKLIB in ECEF*/
void rtklibOdomHandler(const nav_msgs::Odometry::ConstPtr& odomIn) {//, Eigen::Vector3d &first_lla_pvt, Eigen::Vector3d &first_xyz_ecef_pvt, std::vector<double> &pvt_time, 
                        // std::vector<Eigen::Vector3d> &pvt_holder, std::vector<int> &diff_holder, std::vector<int> &float_holder) { // 
    if(RTKinLocal)
    {
        // Eigen::Matrix<double, 3,1> ENU_ref;
        // ENU_ref << 0, 0, 0;
        // ENU_ref = m_GNSS_Tools.ecef2llh(anc_ecef);
        // Eigen::Matrix<double, 3, 1> ENU;
        Eigen::Matrix<double, 3, 1> ECEF;
        ECEF<<odomIn->pose.pose.position.x, odomIn->pose.pose.position.y, odomIn->pose.pose.position.z;
        // ENU = m_GNSS_Tools.ecef2enu(ENU_ref, ECEF);
        // if (fabs(ENU(2) > 300)) return;

        // if (fabs(fabs(ENU(0)) - fabs(m_pose_enu(0))) > 300 ||
            // fabs(fabs(ENU(1)) - fabs(m_pose_enu(1))) > 300 ||
            // fabs(fabs(ENU(2)) - fabs(m_pose_enu(2))) > 300) return;

        nav_msgs::Odometry gnssOdom;
        gnssOdom.header.stamp = odomIn->header.stamp;
        gnssOdom.pose.pose.position.x = ECEF(0); // ENU(0);
        gnssOdom.pose.pose.position.y = ECEF(1); // ENU(1);
        gnssOdom.pose.pose.position.z = ECEF(2); // ENU(2);
        gnssOdom.pose.covariance[0] = odomIn->pose.covariance[0];
        gnssOdom.pose.covariance[1] = odomIn->pose.covariance[1];
        gnssOdom.pose.covariance[2] = odomIn->pose.covariance[2];

        GNSSQueue.push_back(gnssOdom);
        // m_pose_enu = ENU;

        // Eigen::Vector3d lla; //, llh;
        // lla << lat, lon, alt;
        //   llh << lon, lat, alt;
        // if (pvt_time.empty())
        // {
        //     first_lla_pvt = ecef2geo(ECEF); // m_GNSS_Tools.ecef2llh(ECEF)
        //     // std::cout << "first lla:" << first_lla_pvt.transpose() << std::endl;
        //     first_xyz_ecef_pvt = ECEF; // = geo2ecef(lla); // m_GNSS_Tools.llh2ecef(llh);
        //     std::cout << "first ecef xyz:" << first_xyz_ecef_pvt.transpose() << std::endl;
        // }
        // // Eigen::Vector3d xyz_ecef = geo2ecef(lla); // m_GNSS_Tools.llh2ecef(llh);
        // //   Eigen::Vector3d first_lla_pvt_ = m_GNSS_Tools.ecef2llh(first_xyz_ecef_pvt);
        // Eigen::Vector3d xyz_enu = ecef2enu(first_lla_pvt, ECEF - first_xyz_ecef_pvt); // m_GNSS_Tools.
        // pvt_time.push_back(odomIn->header.stamp.toSec());
        // pvt_holder.push_back(xyz_enu);
        // diff_holder.push_back(1);
        // float_holder.push_back(2);
        // std::cout << "pvt_holder size:" << pvt_holder.size() << std::endl;
        /* publish the pose from the RTK initialization */
        // nav_msgs::Odometry odom_mapping; // odom from tc fusion and lc fusion

        // nav_msgs::Odometry odom_mapping_rtk_ini;
        // odom_mapping_rtk_ini = odom_mapping;
        // odom_mapping_rtk_ini.pose.pose.position.x = m_pose_enu(0);
        // odom_mapping_rtk_ini.pose.pose.position.y = m_pose_enu(1);
        // odom_mapping_rtk_ini.pose.pose.position.z = m_pose_enu(2);
        // pub_odom_rtk_ini.publish(odom_mapping_rtk_ini);
        // rtk_ini_enu_path.header = odom_mapping_rtk_ini.header;
        // geometry_msgs::PoseStamped enu_pose_msg;
        // enu_pose_msg.header = rtk_ini_enu_path.header;
        // enu_pose_msg.pose.position.x = m_pose_enu(0);
        // enu_pose_msg.pose.position.y = m_pose_enu(1);
        // enu_pose_msg.pose.position.z = m_pose_enu(2);

        // rtk_ini_enu_path.poses.push_back(enu_pose_msg);
        // pub_path_rtk_ini.publish(rtk_ini_enu_path);
    }
}

/* subscribe the gnss range/doppler measurements from rover */
void rtklib_gnss_meas_callback(const nlosExclusion::GNSS_Raw_ArrayConstPtr &meas_msg, std::queue<std::vector<ObsPtr>> &gnss_meas_vec) //
{
    // nlosExclusion::GNSS_Raw_Array meas_msg_valid;
    /* calculate the integer GNSS second to find station GNSS measurements */
    // std::cout << "check size:" << meas_msg->GNSS_Raws.size() << std::endl;
    if (meas_msg->GNSS_Raws.size() < 1) return;
    gtime_t gps_time_ = gpst2time(meas_msg->GNSS_Raws[0].GNSS_week, meas_msg->GNSS_Raws[0].GNSS_time);
    double gps_time = time2sec(gps_time_);
    // int integerGNSSTime = round((gps_time - 1600000000 - timeshift_IMUtoGNSS)*10);

    std::vector<ObsPtr> gnss_meas;
    int length = meas_msg->GNSS_Raws.size();
    if(length == 0)
    {
        return;
    }  

    double curGNSSSec = meas_msg->GNSS_Raws[0].GNSS_time;

    nlosExclusion::GNSS_Raw_Array closest_gnss_data; // gnss from receiver
    closest_gnss_data = *meas_msg;
    // nlosExclusion::GNSS_Raw_Array st_gnss_data; // gnss from station

    /* try to find the station GNSS measurements */
    // int integerGNSSTimeTmp = integerGNSSTime;
    // std::map<int, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;

    /* rover GNSS data to map (RCV denotes receiver) */
    // std::map<int, nlosExclusion::GNSS_Raw> epochRcvGNSSMap; // array to map
    // gnssRawArray2map(closest_gnss_data, epochRcvGNSSMap);

    /* find the DD measurements related observation
        * please refer to GraphGNSSLib
        * Wen Weisong., Hsu, Li-Ta.* Towards Robust GNSS Positioning and Real-Time Kinematic Using Factor Graph Optimization, ICRA 2021, Xi'an, China.
    */
    // int sv_cnt = st_gnss_data.GNSS_Raws.size();
    /*u_master_sv: user to master 
    *u_iSV: user to ith satellite
    *r_master_sv: reference to master satellite
    */

    /* index the rcv gnss */
    for (size_t i = 0; i < length; i++)
    {
        ObsPtr obs(new Obs());
        
        /* original custimized msg: single satellite */
        nlosExclusion::GNSS_Raw data = meas_msg->GNSS_Raws[i];
        int satPrn = data.prn_satellites_index;
        int prn_;
        int sys_ = satsys_rtk(satPrn, &prn_);
        int sat_num = satno_rtk(sys_, prn_);
        // std::cout << sat_num << ";" << satPrn << ";" << sat_no(sys_, prn_) << ";" << sys_ << ";" << satsys(satPrn, NULL) << std::endl;
        if (sys_ != SYS_GPS && sys_ != SYS_BDS && sys_ != SYS_GAL && sys_ != SYS_GLO) continue;
        // if (sys_ == SYS_GPS || sys_ == SYS_GAL) obs->freqs.push_back(FREQ1);
        // if (sys_ == SYS_BDS) obs->freqs.push_back(FREQ1_BDS);
        // if (sys_ == SYS_GLO) obs->freqs.push_back(FREQ1_GLO);
        // std::cout << prn_ << ";" << sys_ << std::endl;
        // if (satPrn > 127) continue;
        // if(!epochStGNSSMap.count(satPrn)) {
        //     continue;
        // }
        // meas_msg_valid.GNSS_Raws.push_back(data);
        // if(data.pseudorange<100) continue;

        /* receiver to ith satellite */
        obs->time       = gpst2time(data.GNSS_week, data.GNSS_time);
        obs->sat        = sat_no(sys_, prn_); // data.prn_satellites_index; //  
        obs->freqs.push_back(LIGHT_SPEED / data.lamda);
        // std::cout << obs->freqs[i] << ";" << FREQ1 << ";" << FREQ1_BDS << std::endl;
        // uint32_t sys = satsys_rtk(obs->sat, NULL);
        // std::cout << obs->sat << ";" << sys << ";" << SYS_GPS << ";" << SYS_BDS << std::endl;
        obs->CN0.push_back(data.snr);
        // printf("sat:%d;prn:%d;snr:%f\n", obs->sat, prn_, data.snr);
        obs->LLI.push_back(0);
        obs->code.push_back(1);
        // if ((lam_L1=nav->lam[obs[s_i].sat-1][0])>0.0) {
            // dion*=SQR(lam_L1/lam_carr[0]);
        // }
        obs->psr.push_back(data.raw_pseudorange); //pseudorange);
        obs->psr_std.push_back(100.0 / data.snr);
        obs->cp.push_back(data.carrier_phase); // / data.lamda);
        obs->cp_std.push_back(0.10);
        obs->dopp.push_back(data.doppler);
        obs->dopp_std.push_back(20 / data.snr);

        uint8_t ds = *(std::to_string(data.slip).c_str());
        obs->status.push_back(ds);
        // printf("sat:%d;prn:%d;slip:%d\n", obs->sat, prn_, data.slip);
        // obs->status.push_back(*(std::to_string(1).c_str()));
        //set carr phase cycle slip flag
        // if (obs->DD_car) {
        //     uint8_t ds = *(std::to_string(data.slip).c_str());
        //     obs->status.push_back(ds);
        // }

        /* measurements */
        // obs->Uvisable = 1;
        // obs->Usnr = data.snr;
        // obs->Upseudorange = data.pseudorange;
        // obs->Uraw_pseudorange = data.raw_pseudorange;
        // obs->Ucarrier_phase = data.carrier_phase;
        // obs->Ulamda = data.lamda;
        // obs->Uprn_satellites_index = data.prn_satellites_index;

        // obs->Uelevation = data.elevation;
        // obs->Uazimuth = data.azimuth;
        // obs->Uerr_iono = data.err_iono;
        // obs->Uerr_tropo = data.err_tropo;
        // obs->Usat_clk_err = data.sat_clk_err;
        Eigen::Vector3d sat_pos_ecef;
        sat_pos_ecef << data.sat_pos_x, data.sat_pos_y, data.sat_pos_z;
        sat_pos.push_back(sat_pos_ecef);
        // obs->Usat_pos_x = data.sat_pos_x;
        // obs->Usat_pos_y = data.sat_pos_y;
        // obs->Usat_pos_z = data.sat_pos_z;
        // obs->Uttx = data.ttx;
        // obs->Uvel_x = data.vel_x;
        // obs->Uvel_y = data.vel_y;
        // obs->Uvel_z = data.vel_z;
        // obs->Udt = data.dt;
        // obs->Uddt = data.ddt;
        // obs->Utgd = data.tgd;

        // calVar(obs); // calculate covariance for four satellite measurements
        
        gnss_meas.push_back(obs);
    }

    if (gnss_meas.size() == 0) return;
    // /* form the received gnss sginal vectors */
    // gnss_raw_map[integerGNSSTimeTmp] = meas_msg_valid;

    latest_gnss_time_u = time2sec(gnss_meas[0]->time);

    m_buf.lock();
    /* make sure the order of GNSS data is correct */
    if (latest_gnss_time_u > last_gnss_time)
    {
        gnss_meas_vec.push(gnss_meas);
    }
    else
    {
            std::cout<<"GNSS measurements in disorder:   " << latest_gnss_time_u - last_gnss_time<<"\n";
    }

    last_gnss_time = latest_gnss_time_u;
    
    m_buf.unlock();
}

void GtfromTXT_URBAN(const std::string &gt_filepath, std::vector<Eigen::Vector4d> &gt)
{
    // uint32_t gpst_leap_seconds = static_cast<uint32_t>(-1);
    std::ifstream gt_file(gt_filepath);
    std::string line;
    std::getline(gt_file, line);
    std::getline(gt_file, line);
    // while(std::getline(gt_file, line))
    // {
    //     if (line.find("END OF HEADER") != std::string::npos)
    //         break;
    // }

    while(std::getline(gt_file, line))
    {
        std::stringstream ss(line);
        std::string token;
        int i = 0;
        double week, time, d, m, s;
        std::getline(ss, token, ' ');
        std::getline(ss, token, ' ');
        week = std::stof(token);
        // std::cout << "week:" << week << std::endl;
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        time = std::stof(token);
        // std::cout << "time:" << time << std::endl;
        gtime_t gt_gps_time = gpst2time(week, time);
        Eigen::Vector4d gt_vec;
        double latest_gnss_time = time2sec(gt_gps_time);
        gt_vec(0) = latest_gnss_time;
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        d = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        m = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::cout << "line 286:" << token << std::endl;
        s = std::stof(token); //.c_str());
        // std::cout << "line 287:" << d << ";" << m << ";" << s << std::endl;
        gt_vec(1) = d + m / 60 + s / 3600; // std::atof(token.c_str()); // line.at(3); // la
        // std::cout << "line 289:" << gt_vec(1) << std::endl;
        std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        d = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        m = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        s = std::stof(token); //.c_str());
        // std::cout << "line 287:" << d << ";" << m << ";" << s << std::endl;
        gt_vec(2) = d + m / 60 + s / 3600; // std::atof(token.c_str()); // line.at(4); // long
        // std::cout << "line 292:" << gt_vec(2) << std::endl;
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        gt_vec(3) = std::atof(token.c_str()); // line.at(5); // al
        // std::cout << "line 295:" << gt_vec(3) << std::endl;

        gt.push_back(gt_vec);
    }
    std::cout << "gt size:" << gt.size() << std::endl;  
    gt_file.close();
}

void PPPfromTXT(const std::string &ppp_filepath, std::vector<Eigen::Matrix<double, 7, 1>> &ppp_sol, std::vector<Eigen::Vector4d> &ppp_ecef)
{
    // uint32_t gpst_leap_seconds = static_cast<uint32_t>(-1);
    std::ifstream ppp_file(ppp_filepath);
    std::string line;
    // std::getline(gt_file, line);
    // while(std::getline(gt_file, line))
    // {
    //     if (line.find("END OF HEADER") != std::string::npos)
    //         break;
    // }

    while(std::getline(ppp_file, line))
    {
        if (line.find("%") != std::string::npos) continue;
        std::stringstream ss(line);
        std::string token;
        // std::getline(ss, token, ' ');
        // int i = 0;
        double week, time;
        // std::getline(ss, token, ' ');
        std::getline(ss, token, ' ');
        week = std::stof(token);
        // std::cout << "week:" << week << std::endl;
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        time = std::stof(token);
        // std::cout << "time:" << time << std::endl;
        gtime_t gt_gps_time = gpst2time(week, time);
        Eigen::Vector4d gt_vec;
        Eigen::Matrix<double, 7, 1> sol_vec;
        double latest_gnss_time = time2sec(gt_gps_time);
        gt_vec(0) = latest_gnss_time;
        sol_vec(0) = latest_gnss_time;
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        double ecef_x = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        double ecef_y = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::cout << "line 286:" << token << std::endl;
        double ecef_z = std::stof(token); //.c_str());
        Eigen::Vector3d ecef;
        ecef << ecef_x, ecef_y, ecef_z;

        for (size_t i = 0; i < 3; i++)
        {
            std::getline(ss, token, ' ');
            while (token.empty())
            {
                std::getline(ss, token, ' ');
            }
        }
        sol_vec(4) = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
                std::getline(ss, token, ' ');
        }
        sol_vec(5) = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
                std::getline(ss, token, ' ');
        }
        sol_vec(6) = std::stof(token); //.c_str());
        // std::cout << "line 287:" << d << ";" << m << ";" << s << std::endl;
        gt_vec.segment<3>(1) = ecef; 
        ppp_ecef.push_back(gt_vec);
        if (first_pvt)
        {
            first_lla_pvt = ecef2geo(ecef); // m_GNSS_Tools.ecef2llh(ECEF)
            first_xyz_ecef_pvt = ecef;
            first_pvt = false;
        }
        sol_vec.segment<3>(1) = ecef2enu(first_lla_pvt, ecef - first_xyz_ecef_pvt); 
        ppp_sol.push_back(sol_vec);
    }
    std::cout << "ppp size:" << ppp_sol.size() << std::endl;  
    ppp_file.close();
}

void inputpvt_urbannav(double ts, double lat, double lon, double alt, Eigen::Vector3d &first_lla_pvt, Eigen::Vector3d &first_xyz_ecef_pvt, std::vector<double> &pvt_time, 
                        std::vector<Eigen::Vector3d> &pvt_holder, std::vector<int> &diff_holder, std::vector<int> &float_holder) // 
{
  Eigen::Vector3d lla; //, llh;
  lla << lat, lon, alt;
//   llh << lon, lat, alt;
  if (pvt_time.empty())
  {
    first_lla_pvt = lla;
    // first_lla_pvt << 22+18/60+4.31949/3600, 114+10/60+44.60559/3600, 3.472; // = lla;
    // std::cout << "first lla:" << first_lla_pvt.transpose() << std::endl;
    first_xyz_ecef_pvt = geo2ecef(first_lla_pvt); // geo2ecef(lla); // m_GNSS_Tools.llh2ecef(llh);
    printf("first ecef xyz:%f,%f,%f\n",first_xyz_ecef_pvt(0),first_xyz_ecef_pvt(1),first_xyz_ecef_pvt(2));
  }
  Eigen::Vector3d xyz_ecef = geo2ecef(lla); // m_GNSS_Tools.llh2ecef(llh);
//   Eigen::Vector3d first_lla_pvt_ = m_GNSS_Tools.ecef2llh(first_xyz_ecef_pvt);
  Eigen::Vector3d xyz_enu = ecef2enu(first_lla_pvt, xyz_ecef - first_xyz_ecef_pvt); // m_GNSS_Tools.
  pvt_time.push_back(ts);
  pvt_holder.push_back(xyz_enu);
  diff_holder.push_back(1);
  float_holder.push_back(2);
}

void inputpvt_urbannav_ecef(double ts, double ecef_x, double ecef_y, double ecef_z, Eigen::Vector3d &first_lla_pvt, Eigen::Vector3d &first_xyz_ecef_pvt, std::vector<double> &pvt_time, 
                        std::vector<Eigen::Vector3d> &pvt_holder, std::vector<int> &diff_holder, std::vector<int> &float_holder) // 
{
  Eigen::Vector3d ecef; //, llh;
  ecef << ecef_x, ecef_y, ecef_z;
//   llh << lon, lat, alt;
  if (pvt_time.empty())
  {
    first_xyz_ecef_pvt = ecef; // m_GNSS_Tools.llh2ecef(llh);
    first_lla_pvt = ecef2geo(ecef);
    // std::cout << "first lla:" << first_lla_pvt.transpose() << std::endl;
    printf("first ecef xyz:%f,%f,%f\n",first_xyz_ecef_pvt(0),first_xyz_ecef_pvt(1),first_xyz_ecef_pvt(2));
    // std::cout << "first ecef xyz:" << first_xyz_ecef_pvt.transpose() << std::endl;
  }
//   Eigen::Vector3d xyz_ecef = geo2ecef(lla); // m_GNSS_Tools.llh2ecef(llh);
//   Eigen::Vector3d first_lla_pvt_ = m_GNSS_Tools.ecef2llh(first_xyz_ecef_pvt);
  Eigen::Vector3d xyz_enu = ecef2enu(first_lla_pvt, ecef - first_xyz_ecef_pvt); // m_GNSS_Tools.
  pvt_time.push_back(ts);
  pvt_holder.push_back(xyz_enu); // (xyz_enu);
  diff_holder.push_back(1);
  float_holder.push_back(2);
}

int satno_rtk(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
        case SYS_GPS:
            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
            return prn-MINPRNGPS+1;
        case SYS_GLO:
            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
            return NSATGPS+prn-MINPRNGLO+1;
        case SYS_GAL:
            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
        case SYS_QZS:
            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
        case SYS_BDS:
            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
        case SYS_LEO:
            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNLEO+1;
        case SYS_SBS:
            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATLEO+prn-MINPRNSBS+1;
    }
    return 0;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
int satsys_rtk(int sat, int *prn)
{
    int sys=SYS_NONE;
    if (sat<=0||MAXSAT<sat) sat=0;
    else if (sat<=NSATGPS) {
        sys=SYS_GPS; sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1; 
    }
    else if ((sat-=NSATQZS)<=NSATCMP) {
        sys=SYS_BDS; sat+=MINPRNCMP-1; 
    }
    else if ((sat-=NSATCMP)<=NSATLEO) {
        sys=SYS_LEO; sat+=MINPRNLEO-1; 
    }
    else if ((sat-=NSATLEO)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1; 
    }
    else sat=0;
    if (prn) *prn=sat;
    return sys;
}

void GtfromTXT_M2DGR(const std::string &gt_filepath, std::vector<Eigen::Vector4d> &gt)
{
    // uint32_t gpst_leap_seconds = static_cast<uint32_t>(-1);
    std::ifstream gt_file(gt_filepath);
    std::string line;

    while(std::getline(gt_file, line))
    {
        std::stringstream ss(line);
        std::string token;
        int i = 0;
        double time;
        Eigen::Vector4d gt_vec;

        std::getline(ss, token, ' ');
        time = std::stod(token);
        // std::printf("%f\n", time);
        gt_vec(0) = time;
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        gt_vec(1) = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        gt_vec(2) = std::stof(token); //.c_str());
        std::getline(ss, token, ' ');
        while (token.empty())
        {
            std::getline(ss, token, ' ');
        }
        // std::cout << "line 286:" << token << std::endl;
        gt_vec(3) = std::stof(token); //.c_str());
        // std::cout << "line 287:" << d << ";" << m << ";" << s << std::endl;
        // gt_vec.segment<3>(1) = geo2ecef(gt_vec.segment<3>(1));
        gt.push_back(gt_vec);
    }
    std::cout << "gt size:" << gt.size() << std::endl;  
    gt_file.close();
}

void GtfromTXT_LIVOX(const std::string &gt_filepath, std::vector<Eigen::Vector4d> &gt)
{
    // uint32_t gpst_leap_seconds = static_cast<uint32_t>(-1);
    std::ifstream gt_file(gt_filepath);
    std::string line;

    while(std::getline(gt_file, line))
    {
        if (line.find("%") != std::string::npos) continue;

        std::stringstream ss(line);
        std::string token;
        int i = 0;
        double time;
        Eigen::Vector4d gt_vec;

        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        time = std::stod(token);
        // std::printf("%f\n", time);
        gt_vec(0) = time * 1e-9;
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        while (token.empty())
        {
            std::getline(ss, token, ',');
        }
        // std::getline(ss, token, ' ');
        // std::getline(ss, token, ' ');
        gt_vec(1) = std::stof(token); //.c_str());
        std::getline(ss, token, ',');
        while (token.empty())
        {
            std::getline(ss, token, ',');
        }
        gt_vec(2) = std::stof(token); //.c_str());
        std::getline(ss, token, ',');
        while (token.empty())
        {
            std::getline(ss, token, ',');
        }
        // std::cout << "line 286:" << token << std::endl;
        gt_vec(3) = std::stof(token); //.c_str());
        // std::cout << "line 287:" << d << ";" << m << ";" << s << std::endl;
        // gt_vec.segment<3>(1) = geo2ecef(gt_vec.segment<3>(1));
        gt.push_back(gt_vec);
    }
    std::cout << "gt size:" << gt.size() << std::endl;  
    gt_file.close();
}