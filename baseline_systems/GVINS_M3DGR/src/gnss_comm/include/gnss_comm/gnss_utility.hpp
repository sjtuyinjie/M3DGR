/**
* This file is part of gnss_comm.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* gnss_comm is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* gnss_comm is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with gnss_comm. If not, see <http://www.gnu.org/licenses/>.
*
* As many of the utility functions are adapted from RTKLIB, 
* the license for those part of code is claimed as follows:
* 
* The RTKLIB software package is distributed under the following BSD 2-clause
* license (http://opensource.org/licenses/BSD-2-Clause) and additional two
* exclusive clauses. Users are permitted to develop, produce or sell their own
* non-commercial or commercial products utilizing, linking or including RTKLIB as
* long as they comply with the license.
* 
*         Copyright (c) 2007-2020, T. Takasu, All rights reserved.
*/

#ifndef GNSS_UTILITY_HPP_
#define GNSS_UTILITY_HPP_

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <string>
#include <iomanip>
#include <unistd.h>
#include <cmath>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

#include "gnss_constant.hpp"

namespace gnss_comm
{
    // some of the following functions are adapted from RTKLIB

    /* satellite system+prn/slot number to satellite number ------------------------
    * convert satellite system+prn/slot number to satellite number
    * args   : uint32_t    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
    *          uint32_t    prn       I   satellite prn/slot number
    * return : satellite number (0:error)
    *-----------------------------------------------------------------------------*/
    uint32_t sat_no(uint32_t sys, uint32_t prn);

    /* satellite number to satellite system ----------------------------------------
    * convert satellite number to satellite system
    * args   : uint32_t    sat       I   satellite number (1-MAXSAT)
    *          uint32_t    *prn      IO  satellite prn/slot number (NULL: no output)
    * return : satellite system (SYS_GPS,SYS_GLO,...)
    *-----------------------------------------------------------------------------*/
    uint32_t satsys(uint32_t sat, uint32_t *prn);

    /* convert calendar day/time to time -------------------------------------------
    * convert calendar day/time to gtime_t struct
    * args   : double *ep       I   day/time {year,month,day,hour,min,sec}
    * return : gtime_t struct
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    gtime_t epoch2time(const double *ep);
    /* time to calendar day/time ---------------------------------------------------
    * convert gtime_t struct to calendar day/time
    * args   : gtime_t t        I   gtime_t struct
    *          double *ep       O   day/time {year,month,day,hour,min,sec}
    * return : none
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    void time2epoch(gtime_t t, double *ep);

    /* gps time to time ------------------------------------------------------------
    * convert week and tow in gps time to gtime_t struct
    * args   : uint32_t    week      I   week number in gps time
    *          double      tow       I   time of week in gps time (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2time(uint32_t week, double tow);
    /* time to gps time ------------------------------------------------------------
    * convert gtime_t struct to week and tow in gps time
    * args   : gtime_t      t        I   gtime_t struct
    *          uint32_t    *week     IO  week number in gps time (NULL: no output)
    * return : time of week in gps time (s)
    *-----------------------------------------------------------------------------*/
    double time2gpst(gtime_t t, uint32_t *week);
    /* galileo system time to time -------------------------------------------------
    * convert week and tow in galileo system time (gst) to gtime_t struct
    * args   : int    week      I   week number in gst
    *          double tow       I   time of week in gst (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gst2time(int week, double tow);
    /* time to galileo system time -------------------------------------------------
    * convert gtime_t struct to week and tow in galileo system time (gst)
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in gst (NULL: no output)
    * return : time of week in gst (s)
    *-----------------------------------------------------------------------------*/
    double time2gst(gtime_t t, int *week);
    /* beidou time (bdt) to time ---------------------------------------------------
    * convert week and tow in beidou time (bdt) to gtime_t struct
    * args   : int    week      I   week number in bdt
    *          double tow       I   time of week in bdt (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t bdt2time(int week, double tow);
    /* time to beidouo time (bdt) --------------------------------------------------
    * convert gtime_t struct to week and tow in beidou time (bdt)
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in bdt (NULL: no output)
    * return : time of week in bdt (s)
    *-----------------------------------------------------------------------------*/
    double time2bdt(gtime_t t, int *week);

    /* gpstime to utc --------------------------------------------------------------
    * convert gpstime to utc considering leap seconds
    * args   : gtime_t t        I   time expressed in gpstime
    * return : time expressed in utc
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2utc(gtime_t t);

    /* utc to gpstime --------------------------------------------------------------
    * convert utc to gpstime considering leap seconds
    * args   : gtime_t t        I   time expressed in utc
    * return : time expressed in gpstime
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t utc2gpst(gtime_t t);

    /* datetime to Julian day -----------------------------------------------------
    * convert datetime to Julian day
    * args   : std::vector<double> datetime        I   datetime (year, month, day, hour, minute, second)
    * return : Julian day corresponding to the datetime
    * notes  : vaild range: 1900 ~ 2100
    *-----------------------------------------------------------------------------*/
    double julian_day(std::vector<double> datetime);

    /* find the number of leap seconds at the given datetime since GPS start epoch 
    * args   : std::vector<double> datetime        I   datetime (year, month, day, hour, minute, second)
    * return : leap seconds at the given datetime
    *-----------------------------------------------------------------------------*/
    uint32_t leap_seconds_from_GPS_epoch(std::vector<double> datetime);

    /* time to day of year ---------------------------------------------------------
    * convert time to day of year
    * args   : gtime_t t        I   gtime_t struct
    * return : day of year (days)
    *-----------------------------------------------------------------------------*/
    double time2doy(gtime_t time);
    
    /* time difference between two timestamps -------------------------------------
    * args   : gtime_t t1        I   time 1
    *          gtime_t t2        I   time 2
    * return : time difference between t1 and t2 in seconds, e.g. (t1 - t2)
    *-----------------------------------------------------------------------------*/
    double time_diff(gtime_t t1, gtime_t t2);

    /* add specific seconds to a timestamp -------------------------------------
    * args   : gtime_t t        I   base timestamp
    *          double  sec      I   duration to add
    * return : timestamp which equals to `sec` seconds past t
    *-----------------------------------------------------------------------------*/
    gtime_t time_add(gtime_t t, double sec);

    /* time to timestamp in seconds -----------------------------------------------
    * convert gtime_t struct to timestamp in seconds
    * args   : gtime_t t        I   gtime_t struct
    * return : timestamp (s)
    *-----------------------------------------------------------------------------*/
    double time2sec(gtime_t time);

    /* timestamp to time in seconds -----------------------------------------------
    * convert timestamp to gtime_t struct in seconds
    * args   : timestamp sec        I   timestamp in seconds
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t sec2time(const double sec);

    /* Geodetic coordinate to ECEF coordinate -------------------------------------
    * convert a point in geodetic coordinate to ECEF coordinate
    * args   : lla        I   Eigen::Vector3d Latitude, Longitude, Altitude (degree)
    * return : Corresponding ECEF coordinate
    *-----------------------------------------------------------------------------*/
    Eigen::Vector3d geo2ecef(const Eigen::Vector3d &lla);

    /* ECEF coordinate to Geodetic coordinate ----------------------------------------------
    * convert a point in ECEF coordinate to geodetic coordinate
    * args   : xyz        I   Eigen::Vector3d
    * return : Corresponding geodetic coordinate, Longitude, Latitude, Altitude in degree
    *-------------------------------------------------------------------------------------*/
    Eigen::Vector3d ecef2geo(const Eigen::Vector3d &xyz);

    /* solve Kepler equation  ------------------------------------------------------*/
    double Kepler(const double mk, const double es);

    /* calculate satellite clock drift  -------------------------------------------
    * calculate satellite position in ECEF (GPS, GAL, BDS)
    * args   : gtime_t     curr_time       I   current time
    *          Ephem       ephem           I   satellite ephemeris (GPS, BDS, GAL)
    * return : satellite clock drift in seconds
    *-----------------------------------------------------------------------------*/
    double eph2svdt(const gtime_t &curr_time, const EphemPtr ephem_ptr);

    /* calculate satellite position in ECEF  ----------------------------------------
    * calculate satellite position in ECEF (GPS, GAL, BDS)
    * args   : gtime_t     curr_time       I   current time
    *          Ephem       ephem           I   satellite ephemeris (GPS, BDS, GAL)
    *          double*     svdt            IO  satellite clock drift (NULL: no output)
    * return : satellite position in ECEF frame
    *-----------------------------------------------------------------------------*/
    Eigen::Vector3d eph2pos(const gtime_t &curr_time, const EphemPtr ephem_ptr, double *svdt);

    /* calculate satellite velocity in ECEF  ----------------------------------------
    * calculate satellite velocity in ECEF (GPS, GAL, BDS)
    * args   : gtime_t     curr_time       I   current time
    *          Ephem       ephem           I   satellite ephemeris (GPS, BDS, GAL)
    *          double*     svddt           IO  satellite clock drift rate (NULL: no output)
    * return : satellite velocity in ECEF frame
    *-----------------------------------------------------------------------------*/
    Eigen::Vector3d eph2vel(const gtime_t &curr_time, const EphemPtr ephem, double *svddt);

    /* glonass orbit differential equations --------------------------------------*/
    void deq(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
             Eigen::Vector3d &pos_dot, Eigen::Vector3d &vel_dot);
    
    /* glonass position and velocity by numerical integration (RK4) --------------------*/
    void glo_orbit(double dt, Eigen::Vector3d &pos, Eigen::Vector3d &vel, const Eigen::Vector3d &acc);

    /* glonass ephemeris to satellite clock bias -----------------------------------
    * compute satellite clock bias with glonass ephemeris
    * args   : gtime_t curr_time    I   time by satellite clock (gpst)
    *          geph_t  geph         I   glonass ephemeris
    * return : satellite clock bias (s)
    *-----------------------------------------------------------------------------*/
    double geph2svdt(const gtime_t &curr_time, const GloEphemPtr geph_ptr);

    /* glonass ephemeris to satellite position and clock bias ----------------------
    * compute satellite position and clock bias with glonass ephemeris
    * args   : gtime_t curr_time    I       time (gpst)
    *          geph_t  geph         I       glonass ephemeris
    *          double *svdt         IO      output satellite clock bias in second (NULL: no output)
    * return : satellite position in ECEF      Eigen::Vector3d
    *-----------------------------------------------------------------------------*/
    Eigen::Vector3d geph2pos(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *svdt);

    /* glonass ephemeris to satellite position and clock bias ----------------------
    * compute satellite velocity and clock bias change rate with glonass ephemeris
    * args   : gtime_t curr_time    I       time (gpst)
    *          geph_t  geph         I       glonass ephemeris
    *          double *svddt        IO      output satellite clock bias change rate(s/s) (NULL: no output)
    * return : satellite velocity in ECEF      Eigen::Vector3d
    *-----------------------------------------------------------------------------*/
    Eigen::Vector3d geph2vel(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *svddt);

    /* satellite azimuth/elevation angle -------------------------------------------
    * compute satellite azimuth/elevation angle
    * args   : Eigen::Vector3d rev_pos  I   receiver position in ECEF
    *          Eigen::Vector3d sat_pos  I   satellilte position in ECEF
    *          double*         azel     O   azimuth/elevation {az,el} (rad) (NULL: no output)
    *                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
    *-----------------------------------------------------------------------------*/
    void sat_azel(const Eigen::Vector3d &rev_pos, const Eigen::Vector3d &sat_pos, double *azel);

    /* transform ecef vector to local tangental coordinate -------------------------
    * transform ecef vector to local tangental coordinate
    * args   : Eigen::Vector3d pos_lla      I   geodetic position {lat,lon} (degree)
    *          Eigen::Vector3d v_ecef       I   vector in ecef coordinate {x,y,z}
    * return : vector in local tangental coordinate {e,n,u}
    *-----------------------------------------------------------------------------*/
    Eigen::Vector3d ecef2enu(const Eigen::Vector3d &pos_lla, const Eigen::Vector3d &v_ecef);

    /* calculate rotation from ENU frame to ECEF via reference geodetic coordinate  ---
    * args   : Eigen::Vector3d ref_geo      I   reference position in geodetic frame
    * return : rotation matrix from ENU frame to ECEF frame
    *-----------------------------------------------------------------------------*/
    Eigen::Matrix3d geo2rotation(const Eigen::Vector3d &ref_geo);

    /* calculate rotation from ENU frame to ECEF via reference ECEF coordinate  ---
    * args   : Eigen::Vector3d ref_ecef      I   reference position in ECEF
    * return : rotation matrix from ENU frame to ECEF frame
    *-----------------------------------------------------------------------------*/
    Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d &ref_ecef);

    /* calculate signal delay caused by troposphere ------------------------------
    * calculate signal delay caused by troposphere
    * args   : gtime_t time                 I   time (gpst)
    * args   : Eigen::Vector3d rev_lla      I   receiver position in geodetic coordinate
    *          double*         azel         I   satellite azimuth/elevation angle in radius
    * return : delay expressed by light travel distance (m)
    *-----------------------------------------------------------------------------*/
    double calculate_trop_delay(gtime_t time, const Eigen::Vector3d &rev_lla, const double *azel);
                            
    /* calculate signal delay caused by ionosphere ------------------------------
    * calculate signal delay caused by ionosphere
    * args   : gtime_t              time            I   time (gpst)
    *          std::vector<double>  ion_parameters  I   ionosphere parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    *          Eigen::Vector3d      rev_lla         I   receiver position in LLA (lat, lon, hgt) (deg, m)
    *          double*              azel            I   satellite azimuth/elevation angle (rad)
    * return : ionospheric delay (L1) (m)
    *-----------------------------------------------------------------------------*/
    double calculate_ion_delay(gtime_t t, const std::vector<double> &ion_parameters, 
                               const Eigen::Vector3d &rev_lla, const double *azel);

    /* satellite RINEX identifier from RTKLIB satellite id ------------------------------
    * args   : uint32_t             sat_no          I   satellite id in RTKLIB
    * return : satellite RINEX identifier
    *-----------------------------------------------------------------------------*/
    std::string sat2str(uint32_t sat_no);

    /* satellite RINEX identifier to RTKLIB satellite id ------------------------------
    * args   : std::string             &sat_no      I   satellite identifier in RINEX
    * return : satellite id in RTKLIB
    *-----------------------------------------------------------------------------*/
    uint32_t str2sat(const std::string &sat_str);

    /* Get L1 frequency  ---------------------------------------------------------
    * args   : ObsPtr&             obs      I   GNSS observation
    *          int*                l1_idx   I/O L1 frequency index (NULL if not required)
    * return : L1 frequency (negative if L1 frequency not exists)
    *-----------------------------------------------------------------------------*/
    double L1_freq(const ObsPtr &obs, int *l1_idx);

    /* execute the given command  ---------------------------------------------------------
    * args   : std::string    cmd        I   command to execute
    * return : execute result 
    *-------------------------------------------------------------------------------------*/
    std::string exec(const std::string &cmd);

}   // namespace gnss_comm

#endif

