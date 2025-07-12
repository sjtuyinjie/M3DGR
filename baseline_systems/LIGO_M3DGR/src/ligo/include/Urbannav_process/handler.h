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

#include <nav_msgs/Odometry.h>
#include <Urbannav_process/nlosExclusion/GNSS_Raw_Array.h>
#include <Urbannav_process/nlosExclusion/GNSS_Raw.h>
#include <Urbannav_process/nlosExclusion/GNSS_Raw_mf.h>

#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <gnss_comm/gnss_spp.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <mutex>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <math.h>
#include <deque>
#include <queue>
#include "gnss_tools.h"
#include <fstream>
#include <sstream>
// #include <iostream>
// #include <string>
#define process_ppp

#ifdef process_ppp
    extern std::vector<Eigen::Vector4d> ppp_ecef;
    extern std::vector<Eigen::Matrix<double, 7, 1>> ppp_sol;
    extern Eigen::Vector3d first_lla_pvt, first_xyz_ecef_pvt;
    extern bool first_pvt;
#endif

using namespace gnss_comm;

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

// #ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   24                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
// #else
// #define MINPRNGLO   0
// #define MAXPRNGLO   0
// #define NSATGLO     0
// #define NSYSGLO     0
// #endif
// #ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   30                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
// #else
// #define MINPRNGAL   0
// #define MAXPRNGAL   0
// #define NSATGAL     0
// #define NSYSGAL     0
// #endif
// #ifdef ENAQZS
// #define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
// #define MAXPRNQZS   199                 /* max satellite PRN number of QZSS */
// #define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
// #define MAXPRNQZS_S 189                 /* max satellite PRN number of QZSS SAIF */
// #define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
// #define NSYSQZS     1
// #else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
// #endif
// #ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   35                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
// #else
// #define MINPRNCMP   0
// #define MAXPRNCMP   0
// #define NSATCMP     0
// #define NSYSCMP     0
// #endif
// #ifdef ENALEO
// #define MINPRNLEO   1                   /* min satellite sat number of LEO */
// #define MAXPRNLEO   10                  /* max satellite sat number of LEO */
// #define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
// #define NSYSLEO     1
// #else
#define MINPRNLEO   0
#define MAXPRNLEO   0
#define NSATLEO     0
#define NSYSLEO     0
// #endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSLEO) /* number of systems */

#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATSBS+NSATLEO)

extern bool GTinLocal, LCinLocal, RTKinLocal; // visualization

extern double latest_gnss_time_u;
extern double last_gnss_time;
extern std::mutex m_buf;
extern GNSS_Tools m_GNSS_Tools;
extern Eigen::Vector3d m_pose_enu; // pose in enu from initialization
extern std::deque<nav_msgs::Odometry> GNSSQueue;

extern std::vector<Eigen::Vector3d> sat_pos;
extern std::vector<Eigen::Vector3d> sat_vel;

void rtklibOdomHandler(const nav_msgs::Odometry::ConstPtr& odomIn); //, Eigen::Vector3d &first_lla_pvt, Eigen::Vector3d &first_xyz_ecef_pvt, std::vector<double> &pvt_time, 
                        // std::vector<Eigen::Vector3d> &pvt_holder, std::vector<int> &diff_holder, std::vector<int> &float_holder);
void rtklib_gnss_meas_callback(const nlosExclusion::GNSS_Raw_ArrayConstPtr &meas_msg, std::queue<std::vector<ObsPtr>> &gnss_meas_vec);
bool gnssRawArray2map(nlosExclusion::GNSS_Raw_Array gnss_data, std::map<int, nlosExclusion::GNSS_Raw> &epochGnssMap);
bool calVar(ObsPtr &obs);
double eleSRNVarCal(double ele, double snr);
void GtfromTXT_URBAN(const std::string &gt_filepath, std::vector<Eigen::Vector4d> &gt);
void GtfromTXT_M2DGR(const std::string &gt_filepath, std::vector<Eigen::Vector4d> &gt);
void GtfromTXT_LIVOX(const std::string &gt_filepath, std::vector<Eigen::Vector4d> &gt);
void inputpvt_urbannav(double ts, double lat, double lon, double alt, Eigen::Vector3d &first_lla_pvt, Eigen::Vector3d &first_xyz_ecef_pvt, std::vector<double> &pvt_time, 
                        std::vector<Eigen::Vector3d> &pvt_holder, std::vector<int> &diff_holder, std::vector<int> &float_holder);
void inputpvt_urbannav_ecef(double ts, double ecef_x, double ecef_y, double ecef_z, Eigen::Vector3d &first_lla_pvt, Eigen::Vector3d &first_xyz_ecef_pvt, std::vector<double> &pvt_time, 
                        std::vector<Eigen::Vector3d> &pvt_holder, std::vector<int> &diff_holder, std::vector<int> &float_holder);
int satno_rtk(int sys, int prn);
int satsys_rtk(int sat, int *prn);
void PPPfromTXT(const std::string &ppp_filepath, std::vector<Eigen::Matrix<double, 7, 1>> &ppp_sol, std::vector<Eigen::Vector4d> &ppp_ecef);