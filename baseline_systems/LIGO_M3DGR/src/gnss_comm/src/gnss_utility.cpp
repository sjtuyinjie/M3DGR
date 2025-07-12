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

#include "gnss_utility.hpp"

namespace gnss_comm
{
    // some of the following functions are adapted from RTKLIB

    const static double gpst0[] = {1980,1,6,0,0,0}; /* gps time reference */
    const static double gst0 [] = {1999,8,22,0,0,0}; /* galileo system time reference */
    const static double bdt0 [] = {2006,1,1,0,0,0}; /* beidou time reference */

    /* satellite system+prn/slot number to satellite number ------------------------
    * convert satellite system+prn/slot number to satellite number
    * args   : uint32_t    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
    *          uint32_t    prn       I   satellite prn/slot number
    * return : satellite number (0:error)
    *-----------------------------------------------------------------------------*/
    uint32_t sat_no(uint32_t sys, uint32_t prn)
    {
        if (prn == 0) return 0;
        switch (sys) {
            case SYS_GPS:
                if (prn < MIN_PRN_GPS || prn > MAX_PRN_GPS) return 0;
                return prn-MIN_PRN_GPS+1;
            case SYS_GLO:
                if (prn < MIN_PRN_GLO || prn > MAX_PRN_GLO) return 0;
                return N_SAT_GPS+prn-MIN_PRN_GLO+1;
            case SYS_GAL:
                if (prn < MIN_PRN_GAL || prn > MAX_PRN_GAL) return 0;
                return N_SAT_GPS+N_SAT_GLO+prn-MIN_PRN_GAL+1;
            case SYS_BDS:
                if (prn < MIN_PRN_BDS || prn > MAX_PRN_BDS) return 0;
                return N_SAT_GPS+N_SAT_GLO+N_SAT_GAL+prn-MIN_PRN_BDS+1;
        }
        return 0;
    }

    /* satellite number to satellite system ----------------------------------------
    * convert satellite number to satellite system
    * args   : uint32_t    sat       I   satellite number (1-MAXSAT)
    *          uint32_t    *prn      IO  satellite prn/slot number (NULL: no output)
    * return : satellite system (SYS_GPS,SYS_GLO,...)
    *-----------------------------------------------------------------------------*/
    uint32_t satsys(uint32_t sat, uint32_t *prn)
    {
        uint32_t sys = SYS_NONE;
        if (sat <= 0 || sat > MAX_SAT) sat = 0;
        else if (sat <= N_SAT_GPS) {
            sys = SYS_GPS; sat += MIN_PRN_GPS-1;
        }
        else if ((sat-=N_SAT_GPS) <= N_SAT_GLO) {
            sys = SYS_GLO; sat += MIN_PRN_GLO-1;
        }
        else if ((sat-=N_SAT_GLO) <= N_SAT_GAL) {
            sys = SYS_GAL; sat += MIN_PRN_GAL-1;
        }
        else if ((sat-=N_SAT_GAL) <= N_SAT_BDS) {
            sys = SYS_BDS; sat += MIN_PRN_BDS-1; 
        }
        else sat = 0;
        if (prn) *prn = sat;
        return sys;
    }

    /* convert calendar day/time to time -------------------------------------------
    * convert calendar day/time to gtime_t struct
    * args   : double *ep       I   day/time {year,month,day,hour,min,sec}
    * return : gtime_t struct
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    gtime_t epoch2time(const double *ep)
    {
        const int doy[] = {1,32,60,91,121,152,182,213,244,274,305,335};
        gtime_t time = {0};
        int days, sec, year=(int)ep[0], mon=(int)ep[1], day=(int)ep[2];
        
        if (year < 1970 || year > 2099 || mon < 1 || mon > 12) return time;
        
        /* leap year if year%4==0 in 1901-2099 */
        days = (year-1970)*365 + (year-1969)/4 + doy[mon-1] + day-2 + (year%4==0&&mon>=3?1:0);
        sec = (int)floor(ep[5]);
        time.time = (time_t)days*86400 + (int)ep[3]*3600 + (int)ep[4]*60 + sec;
        time.sec = ep[5] - sec;
        return time;
    }
    /* time to calendar day/time ---------------------------------------------------
    * convert gtime_t struct to calendar day/time
    * args   : gtime_t t        I   gtime_t struct
    *          double *ep       O   day/time {year,month,day,hour,min,sec}
    * return : none
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    void time2epoch(gtime_t t, double *ep)
    {
        const int mday[] = { /* # of days in a month */
            31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
            31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
        };
        int days, sec, mon, day;
        
        /* leap year if year%4==0 in 1901-2099 */
        days = (int)(t.time / 86400);
        sec = (int)(t.time - (time_t)days*86400);
        for (day=days%1461, mon=0; mon<48; mon++) {
            if (day >= mday[mon]) day -= mday[mon]; else break;
        }
        ep[0] = 1970 + days/1461*4 + mon/12;
        ep[1] = mon%12 + 1;
        ep[2] = day + 1;
        ep[3] = sec / 3600;
        ep[4] = sec%3600 / 60;
        ep[5] = sec%60 + t.sec;
    }

    /* gps time to time ------------------------------------------------------------
    * convert week and tow in gps time to gtime_t struct
    * args   : uint32_t    week      I   week number in gps time
    *          double      tow       I   time of week in gps time (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2time(uint32_t week, double tow)
    {
        gtime_t t = epoch2time(gpst0);
        if (tow < -1E9 || tow > 1E9) tow = 0.0;
        t.time += 86400*7*week + (int)tow;
        t.sec = tow - (int)tow;
        return t;
    }
    /* time to gps time ------------------------------------------------------------
    * convert gtime_t struct to week and tow in gps time
    * args   : gtime_t      t        I   gtime_t struct
    *          uint32_t    *week     IO  week number in gps time (NULL: no output)
    * return : time of week in gps time (s)
    *-----------------------------------------------------------------------------*/
    double time2gpst(gtime_t t, uint32_t *week)
    {
        gtime_t t0 = epoch2time(gpst0);
        time_t sec = t.time - t0.time;
        uint32_t w = (uint32_t)(sec / (86400*7));
        
        if (week) *week = w;
        return (double)(sec - w*86400*7) + t.sec;
    }
    /* galileo system time to time -------------------------------------------------
    * convert week and tow in galileo system time (gst) to gtime_t struct
    * args   : int    week      I   week number in gst
    *          double tow       I   time of week in gst (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gst2time(int week, double tow)
    {
        gtime_t t = epoch2time(gst0);
        
        if (tow < -1E9 || tow > 1E9) tow = 0.0;
        t.time += 86400*7*week + (int)tow;
        t.sec = tow - (int)tow;
        return t;
    }
    /* time to galileo system time -------------------------------------------------
    * convert gtime_t struct to week and tow in galileo system time (gst)
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in gst (NULL: no output)
    * return : time of week in gst (s)
    *-----------------------------------------------------------------------------*/
    double time2gst(gtime_t t, int *week)
    {
        gtime_t t0 = epoch2time(gst0);
        time_t sec = t.time-t0.time;
        int w = (int)(sec / (86400*7));
        
        if (week) *week = w;
        return (double)(sec - w*86400*7) + t.sec;
    }
    /* beidou time (bdt) to time ---------------------------------------------------
    * convert week and tow in beidou time (bdt) to gtime_t struct
    * args   : int    week      I   week number in bdt
    *          double tow       I   time of week in bdt (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t bdt2time(int week, double tow)
    {
        gtime_t t = epoch2time(bdt0);
        
        if (tow < -1E9 || tow > 1E9) tow = 0.0;
        t.time += 86400*7*week + (int)tow;
        t.sec = tow - (int)tow;
        return t;
    }
    /* time to beidouo time (bdt) --------------------------------------------------
    * convert gtime_t struct to week and tow in beidou time (bdt)
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in bdt (NULL: no output)
    * return : time of week in bdt (s)
    *-----------------------------------------------------------------------------*/
    double time2bdt(gtime_t t, int *week)
    {
        gtime_t t0 = epoch2time(bdt0);
        time_t sec = t.time - t0.time;
        int w = (int)(sec / (86400*7));
        
        if (week) *week = w;
        return (double)(sec - w*86400*7) + t.sec;
    }

    /* gpstime to utc --------------------------------------------------------------
    * convert gpstime to utc considering leap seconds
    * args   : gtime_t t        I   time expressed in gpstime
    * return : time expressed in utc
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2utc(gtime_t t)
    {
        gtime_t tu;
        int i;
        
        for (i=0;leaps[i][0]>0;i++) 
        {
            tu=time_add(t,leaps[i][6]);
            if (time_diff(tu,epoch2time(leaps[i]))>=0.0) return tu;
        }
        return t;
    }

    /* utc to gpstime --------------------------------------------------------------
    * convert utc to gpstime considering leap seconds
    * args   : gtime_t t        I   time expressed in utc
    * return : time expressed in gpstime
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t utc2gpst(gtime_t t)
    {
        int i;
        
        for (i=0;leaps[i][0]>0;i++) 
        {
            if (time_diff(t,epoch2time(leaps[i]))>=0.0) return time_add(t,-leaps[i][6]);
        }
        return t;
    }

    double julian_day(std::vector<double> datetime)
    {
        LOG_IF(FATAL, datetime.size() != 6) << "datetime should contain 6 fields";
        LOG_IF(FATAL, datetime[0] < 1901 || datetime[0] > 2099) << datetime[0] << " should within range 1900 ~ 2100";

        if (datetime[1] <= 2)
        {
            datetime[1] += 12;
            datetime[0] -= 1;
        }
        double julian_day = floor(365.25*datetime[0]) + floor(30.6001*(datetime[1]+1)) - 
                            15 + 1720996.5 + datetime[2] + datetime[3] / 24 + 
                            datetime[4] / 60 / 24 + datetime[5] / 3600 / 24;
        return julian_day;
    }

    uint32_t leap_seconds_from_GPS_epoch(std::vector<double> datetime)
    {
        gtime_t given_t = epoch2time(&(datetime[0]));
        for (size_t i = 0; leaps[i][0] > 0; ++i)
        {
            gtime_t tu = time_add(given_t, leaps[i][6]);
            if (time_diff(tu, epoch2time(leaps[i])) >= 0.0)
                return static_cast<uint32_t>(-leaps[i][6]);
        }
        return static_cast<uint32_t>(-1);
    }

    double time2doy(gtime_t time)
    {
        double ep[6];
        time2epoch(time, ep);
        ep[1] = ep[2] = 1.0;
        ep[3] = ep[4] = ep[5] = 0.0;
        return time_diff(time, epoch2time(ep))/86400.0 + 1.0;
    }

    double time_diff(gtime_t t1, gtime_t t2)
    {
        return difftime(t1.time,t2.time)+t1.sec-t2.sec;
    }

    gtime_t time_add(gtime_t t, double sec)
    {
        t.sec += sec;
        double tt = floor(t.sec);
        t.time += static_cast<int>(tt);
        t.sec -= tt;
        return t;
    }

    double time2sec(gtime_t time)
    {
        return static_cast<double>(time.time) + time.sec;
    }

    gtime_t sec2time(const double sec)
    {
        gtime_t time;
        time.time = floor(sec);
        time.sec = sec - time.time;
        return time;
    }

    Eigen::Vector3d geo2ecef(const Eigen::Vector3d &lla)
    {
        Eigen::Vector3d xyz;
        const double cos_lat = std::cos(lla(0)*D2R);
        const double sin_lat = std::sin(lla(0)*D2R);
        const double N = EARTH_SEMI_MAJOR / std::sqrt(1 - EARTH_ECCE_2*sin_lat*sin_lat);
        xyz.x() = (N + lla(2)) * cos_lat * std::cos(lla(1)*D2R);
        xyz.y() = (N + lla(2)) * cos_lat * std::sin(lla(1)*D2R);
        xyz.z() = (N*(1-EARTH_ECCE_2) + lla(2)) * sin_lat;
        return xyz;
    }

    Eigen::Vector3d ecef2geo(const Eigen::Vector3d &xyz)
    {
        Eigen::Vector3d lla = Eigen::Vector3d::Zero();
        if (xyz.x() == 0 && xyz.y() == 0)
        {
            LOG(ERROR) << "LLA coordinate is not defined if x = 0 and y = 0";
            return lla;
        }

        double e2 = EARTH_ECCE_2;
        double a = EARTH_SEMI_MAJOR;
        double a2 = a * a;
        double b2 = a2 * (1 - e2);
        double b = sqrt(b2);
        double ep2 = (a2 - b2) / b2;
        double p = xyz.head<2>().norm();

        // two sides and hypotenuse of right angle triangle with one angle = theta:
        double s1 = xyz.z() * a;
        double s2 = p * b;
        double h = sqrt(s1 * s1 + s2 * s2);
        double sin_theta = s1 / h;
        double cos_theta = s2 / h;

        // two sides and hypotenuse of right angle triangle with one angle = lat:
        s1 = xyz.z() + ep2 * b * pow(sin_theta, 3);
        s2 = p - a * e2 * pow(cos_theta, 3);
        h = sqrt(s1 * s1 + s2 * s2);
        double tan_lat = s1 / s2;
        double sin_lat = s1 / h;
        double cos_lat = s2 / h;
        double lat = atan(tan_lat);
        double lat_deg = lat * R2D;

        double N = a2 * pow((a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat), -0.5);
        double altM = p / cos_lat - N;

        double lon = atan2(xyz.y(), xyz.x());
        double lon_deg = lon * R2D;
        lla << lat_deg, lon_deg, altM;
        return lla;
    }

    double Kepler(const double mk, const double es)
    {
        double e = mk;
        double ek = 1e6;
        size_t num_iter = 0;
        while (num_iter < MAX_ITER_KEPLER && fabs(e-ek) > EPSILON_KEPLER)
        {
            ek = e;
            e -= (e - es * sin(e) - mk) / (1.0 - es * cos(e));
            ++ num_iter;
        }
        if (num_iter == MAX_ITER_KEPLER)
            LOG(WARNING) << "Kepler equation reached max number iteration";
        
        return ek;
    }

    /* double eph2svdt(const gtime_t &curr_time, const EphemPtr ephem_ptr)
    {
        double tk = time_diff(curr_time, ephem_ptr->toe);
        if (tk > WEEK_SECONDS/2)
            tk -= WEEK_SECONDS;
        else if (tk < -WEEK_SECONDS/2)
            tk += WEEK_SECONDS;

        double n0 = sqrt(MU / pow(ephem_ptr->A, 3));     // mean motion angular velocity (rad/sec)
        double n = n0 + ephem_ptr->delta_n;              // corrected mean motion
        double Mk = ephem_ptr->M0 + n * tk;
        double Ek = Kepler(Mk, ephem_ptr->e);                     // solve Kepler equation for eccentric anomaly

        double dt = time_diff(curr_time, ephem_ptr->toc);
        if (dt > WEEK_SECONDS/2)
            dt -= WEEK_SECONDS;
        else if (dt < -WEEK_SECONDS/2)
            dt += WEEK_SECONDS;
        
        for (size_t i = 0; i < 2; ++i)
            dt -= ephem_ptr->af0 + ephem_ptr->af1*dt + ephem_ptr->af2*dt*dt;
        
        double dtsv = ephem_ptr->af0 + ephem_ptr->af1 * dt + ephem_ptr->af2 * dt * dt;
        double rel_dt = -2.0 * sqrt(MU)/LIGHT_SPEED/LIGHT_SPEED * ephem_ptr->e * sqrt(ephem_ptr->A) * sin(Ek);
        rel_dt = 0;
        dtsv += rel_dt;
        // LOG(INFO) << "dtsv is " << std::setprecision(10) << dtsv;
        return dtsv;
    } */

    double eph2svdt(const gtime_t &curr_time, const EphemPtr ephem_ptr)
    {
        double dt = time_diff(curr_time, ephem_ptr->toc);
        
        for (size_t i = 0; i < 2; ++i)
            dt -= ephem_ptr->af0 + ephem_ptr->af1*dt + ephem_ptr->af2*dt*dt;
        
        double dtsv = ephem_ptr->af0 + ephem_ptr->af1 * dt + ephem_ptr->af2 * dt * dt;
        return dtsv;
    }

    Eigen::Vector3d eph2pos(const gtime_t &curr_time, const EphemPtr ephem_ptr, double *svdt)
    {
        Eigen::Vector3d sv_pos;
        double tk = time_diff(curr_time, ephem_ptr->toe);

        if (tk > WEEK_SECONDS/2)
            tk -= WEEK_SECONDS;
        else if (tk < -WEEK_SECONDS/2)
            tk += WEEK_SECONDS;
        
        // LOG(INFO) << "tk is " << std::setprecision(10) << tk;
        
        if (std::abs(tk) > EPH_VALID_SECONDS)
            LOG(WARNING) << "Ephemeris is not valid anymore";
        
        uint32_t prn;
        uint32_t sys = satsys(ephem_ptr->sat, &prn);
        double mu = MU, earth_omg = EARTH_OMG_GPS;
        switch (sys)
        {
            case SYS_GPS: mu = MU_GPS; earth_omg = EARTH_OMG_GPS; break;
            case SYS_GAL: mu = MU;     earth_omg = EARTH_OMG_GPS; break;
            case SYS_GLO: mu = MU;     earth_omg = EARTH_OMG_GLO; break;
            case SYS_BDS: mu = MU;     earth_omg = EARTH_OMG_BDS; break;
        }

        // LOG(INFO) << "SYS: " << sys << ", prn: " << prn << ", curr_time is " 
        //           << std::setprecision(10) << curr_time.time
        //           << ", and toe is " << ephem_ptr->toe.time;

        double n0 = sqrt(mu / pow(ephem_ptr->A, 3));     // mean motion angular velocity (rad/sec)
        double n = n0 + ephem_ptr->delta_n;              // corrected mean motion
        double Mk = ephem_ptr->M0 + n * tk;
        double Ek = Kepler(Mk, ephem_ptr->e);                     // solve Kepler equation for eccentric anomaly
        double sin_Ek = sin(Ek), cos_Ek = cos(Ek);
        double vk = atan2(sqrt(1 - ephem_ptr->e*ephem_ptr->e) * sin_Ek, cos_Ek - ephem_ptr->e);
        double phi = vk + ephem_ptr->omg;
        double cos_2phi = cos(2 * phi);
        double sin_2phi = sin(2 * phi);

        double delta_uk = ephem_ptr->cus * sin_2phi + ephem_ptr->cuc * cos_2phi;  // latitude correction
        double delta_rk = ephem_ptr->crs * sin_2phi + ephem_ptr->crc * cos_2phi;  // radius correction
        double delta_ik = ephem_ptr->cis * sin_2phi + ephem_ptr->cic * cos_2phi;  // correction to inclination
        double uk = phi + delta_uk;                                             // corrected latitude
        double rk = ephem_ptr->A * (1 - ephem_ptr->e * cos_Ek) + delta_rk;                // corrected radius
        double ik = ephem_ptr->i0 + ephem_ptr->i_dot * tk + delta_ik;             // corrected inclination
        double sin_ik = sin(ik), cos_ik = cos(ik);

        double xk_prime = rk * cos(uk);     // x position in orbital plane
        double yk_prime = rk * sin(uk);     // y position in orbital plane

        double toe_tow = (sys == SYS_BDS ? time2bdt(time_add(ephem_ptr->toe, -14), NULL) : ephem_ptr->toe_tow);

        if (sys == SYS_BDS && prn <= 5)     // BDS GEO satellite
        {
            double OMG_k = ephem_ptr->OMG0 + ephem_ptr->OMG_dot * tk - earth_omg * toe_tow;
            double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
            double xg = xk_prime * cos_OMG_k - yk_prime * cos_ik * sin_OMG_k;
            double yg = xk_prime * sin_OMG_k + yk_prime * cos_ik * cos_OMG_k;
            double zg = yk_prime * sin_ik;
            double sin_o = sin(earth_omg * tk), cos_o = cos(earth_omg * tk);
            sv_pos.x() =  xg * cos_o + yg * sin_o * COS_N5 + zg * sin_o * SIN_N5;
            sv_pos.y() = -xg * sin_o + yg * cos_o * COS_N5 + zg * cos_o * SIN_N5;
            sv_pos.z() = -yg * SIN_N5 + zg * COS_N5;
        }
        else
        {
            double OMG_k = ephem_ptr->OMG0 + (ephem_ptr->OMG_dot - earth_omg) * tk 
                    - earth_omg * toe_tow;
            double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
            sv_pos.x() = xk_prime * cos_OMG_k - yk_prime * cos_ik * sin_OMG_k;
            sv_pos.y() = xk_prime * sin_OMG_k + yk_prime * cos_ik * cos_OMG_k;
            sv_pos.z() = yk_prime * sin_ik;
        }

        double dt = time_diff(curr_time, ephem_ptr->toc);
        double dts = ephem_ptr->af0 + ephem_ptr->af1 * dt + ephem_ptr->af2 * dt * dt;
        // relativity correction
        dts -= 2.0 * sqrt(mu * ephem_ptr->A) * ephem_ptr->e * sin_Ek / LIGHT_SPEED / LIGHT_SPEED;

        if (svdt)  *svdt = dts;
        
        return sv_pos;
    }

    Eigen::Vector3d eph2vel(const gtime_t &curr_time, const EphemPtr ephem_ptr, double *svddt)
    {
        Eigen::Vector3d sv_vel;
        sv_vel.setZero();
        double tk = time_diff(curr_time, ephem_ptr->toe);

        if (tk > WEEK_SECONDS/2)
            tk -= WEEK_SECONDS;
        else if (tk < -WEEK_SECONDS/2)
            tk += WEEK_SECONDS;
        
        // LOG(INFO) << "tk is " << std::setprecision(10) << tk;
        
        if (std::abs(tk) > EPH_VALID_SECONDS)
            LOG(WARNING) << "Ephemeris is not valid anymore";
        
        uint32_t prn;
        uint32_t sys = satsys(ephem_ptr->sat, &prn);
        double mu = MU, earth_omg = EARTH_OMG_GPS;
        switch (sys)
        {
            case SYS_GPS: mu = MU_GPS; earth_omg = EARTH_OMG_GPS; break;
            case SYS_GAL: mu = MU;     earth_omg = EARTH_OMG_GPS; break;
            case SYS_GLO: mu = MU;     earth_omg = EARTH_OMG_GLO; break;
            case SYS_BDS: mu = MU;     earth_omg = EARTH_OMG_BDS; break;
        }
        double n0 = sqrt(mu / pow(ephem_ptr->A, 3));     // mean motion angular velocity (rad/sec)
        double n = n0 + ephem_ptr->delta_n;              // corrected mean motion
        double Mk = ephem_ptr->M0 + n * tk;
        double Ek = Kepler(Mk, ephem_ptr->e);                     // solve Kepler equation for eccentric anomaly
        double sin_Ek = sin(Ek), cos_Ek = cos(Ek);
        double Ek_dot = n / (1 - ephem_ptr->e*cos_Ek);
        double vk_dot = sqrt(1-ephem_ptr->e*ephem_ptr->e) * Ek_dot / (1 - ephem_ptr->e*cos_Ek);
        double vk = atan2(sqrt(1 - ephem_ptr->e*ephem_ptr->e) * sin_Ek, cos_Ek - ephem_ptr->e);
        double phi = vk + ephem_ptr->omg;
        double cos_2phi = cos(2 * phi);
        double sin_2phi = sin(2 * phi);

        double delta_uk_dot = 2 * vk_dot * (ephem_ptr->cus*cos_2phi - ephem_ptr->cuc*sin_2phi);
        double delta_rk_dot = 2 * vk_dot * (ephem_ptr->crs*cos_2phi - ephem_ptr->crc*sin_2phi);
        double delta_ik_dot = 2 * vk_dot * (ephem_ptr->cis*cos_2phi - ephem_ptr->cic*sin_2phi);

        double uk_dot   = vk_dot + delta_uk_dot;
        double rk_dot   = ephem_ptr->A * ephem_ptr->e * Ek_dot * sin_Ek + delta_rk_dot;
        double ik_dot   = ephem_ptr->i_dot + delta_ik_dot;


        double delta_uk = ephem_ptr->cus * sin_2phi + ephem_ptr->cuc * cos_2phi;  // latitude correction
        double delta_rk = ephem_ptr->crs * sin_2phi + ephem_ptr->crc * cos_2phi;  // radius correction
        double delta_ik = ephem_ptr->cis * sin_2phi + ephem_ptr->cic * cos_2phi;  // correction to inclination
        double uk = phi + delta_uk;                                     // corrected latitude
        double rk = ephem_ptr->A * (1 - ephem_ptr->e * cos_Ek) + delta_rk;        // corrected radius
        double ik = ephem_ptr->i0 + ephem_ptr->i_dot * tk + delta_ik;             // corrected inclination
        double sin_ik = sin(ik), cos_ik = cos(ik);

        double sin_uk = sin(uk), cos_uk = cos(uk);
        double xk_prime = rk * cos(uk);     // x position in orbital plane
        double yk_prime = rk * sin(uk);     // y position in orbital plane
        double xk_prime_dot = rk_dot * cos_uk - rk * uk_dot * sin_uk;
        double yk_prime_dot = rk_dot * sin_uk + rk * uk_dot * cos_uk;
        

        double toe_tow = (sys == SYS_BDS ? time2bdt(time_add(ephem_ptr->toe, -14), NULL) : ephem_ptr->toe_tow);

        if (sys == SYS_BDS && prn <= 5)     // BDS GEO satellite
        {
            double OMG_k = ephem_ptr->OMG0 + ephem_ptr->OMG_dot * tk - earth_omg * toe_tow;
            double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
            double OMGk_dot = ephem_ptr->OMG_dot;
            double term1 = xk_prime_dot - yk_prime*OMGk_dot*cos_ik;
            double term2 = xk_prime*OMGk_dot + yk_prime_dot*cos_ik-yk_prime*ik_dot*sin_ik;
            double xg = xk_prime * cos_OMG_k - yk_prime * cos_ik * sin_OMG_k;
            double yg = xk_prime * sin_OMG_k + yk_prime * cos_ik * cos_OMG_k;
            double zg = yk_prime * sin_ik;
            double xg_dot = term1 * cos_OMG_k - term2 * sin_OMG_k;
            double yg_dot = term1 * sin_OMG_k + term2 * cos_OMG_k;
            double zg_dot = yk_prime_dot * sin_ik + yk_prime_dot * ik_dot * cos_ik;
            double sin_o = sin(earth_omg * tk), cos_o = cos(earth_omg * tk);
            double sin_o_dot = earth_omg * cos_o, cos_o_dot = -earth_omg * sin_o;

            sv_vel.x() = xg_dot*cos_o + xg*cos_o_dot + yg_dot*sin_o*COS_N5 + yg*sin_o_dot*COS_N5 + 
                         zg_dot*sin_o*SIN_N5 + zg*sin_o_dot*SIN_N5;
            sv_vel.y() = -xg_dot*sin_o - xg*sin_o_dot + yg_dot*cos_o*COS_N5 + yg*cos_o_dot*COS_N5 + 
                         zg_dot*cos_o*SIN_N5 + zg*cos_o_dot*SIN_N5;
            sv_vel.z() = -yg_dot*SIN_N5 + zg_dot*COS_N5;
        }
        else
        {
            double OMG_k = ephem_ptr->OMG0 + (ephem_ptr->OMG_dot - earth_omg) * tk 
                    - earth_omg * toe_tow;
            double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
            double OMGk_dot = ephem_ptr->OMG_dot - earth_omg;
            double term1 = xk_prime_dot - yk_prime*OMGk_dot*cos_ik;
            double term2 = xk_prime*OMGk_dot + yk_prime_dot*cos_ik-yk_prime*ik_dot*sin_ik;
            sv_vel.x() = term1 * cos_OMG_k - term2 * sin_OMG_k;
            sv_vel.y() = term1 * sin_OMG_k + term2 * cos_OMG_k;
            sv_vel.z() = yk_prime_dot * sin_ik + yk_prime_dot * ik_dot * cos_ik;
        }

        double dt = time_diff(curr_time, ephem_ptr->toc);
        double ddts = ephem_ptr->af1 + 2.0 * ephem_ptr->af2 * dt;
        // relativity correction
        ddts -= 2.0 * sqrt(mu * ephem_ptr->A) * ephem_ptr->e * cos_Ek * Ek_dot / LIGHT_SPEED / LIGHT_SPEED;

        if (svddt)  *svddt = ddts;
        
        return sv_vel;
    }

    void deq(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
             Eigen::Vector3d &pos_dot, Eigen::Vector3d &vel_dot)
    {
        double r2 = pos.squaredNorm(), r3 = r2 * sqrt(r2);
        double omg2 = EARTH_OMG_GLO * EARTH_OMG_GLO;
        
        if (r2 <= 0.0) 
        {
            pos_dot.setZero();
            vel_dot.setZero();
            return;
        }
        /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
        double a = 1.5 * J2_GLO  * MU * EARTH_SEMI_MAJOR_GLO * EARTH_SEMI_MAJOR_GLO / r2 / r3;    /* 3/2*J2*mu*Ae^2/r^5 */
        double b = 5.0 * pos.z()*pos.z() / r2;                                  /* 5*z^2/r^2 */
        double c = -MU / r3 - a * (1.0 - b);                                    /* -mu/r^3-a(1-b) */
        pos_dot.x() = vel.x(); pos_dot.y() = vel.y(); pos_dot.z() = vel.z();
        vel_dot.x() = (c + omg2) * pos.x() + 2.0 * EARTH_OMG_GLO * vel.y() + acc.x();
        vel_dot.y() = (c + omg2) * pos.y() - 2.0 * EARTH_OMG_GLO * vel.x() + acc.y();
        vel_dot.z() = (c - 2.0 * a) * pos.z() + acc.z();
    }

    void glo_orbit(double dt, Eigen::Vector3d &pos, Eigen::Vector3d &vel, const Eigen::Vector3d &acc)
    {
        Eigen::Vector3d pos_dot1, pos_dot2, pos_dot3, pos_dot4;
        Eigen::Vector3d vel_dot1, vel_dot2, vel_dot3, vel_dot4;
        Eigen::Vector3d next_pos, next_vel;

        deq(pos, vel, acc, pos_dot1, vel_dot1);
        next_pos = pos + 0.5 * pos_dot1 * dt;
        next_vel = vel + 0.5 * vel_dot1 * dt;
        deq(next_pos, next_vel, acc, pos_dot2, vel_dot2);
        next_pos = pos + 0.5 * pos_dot2 * dt;
        next_vel = vel + 0.5 * vel_dot2 * dt;
        deq(next_pos, next_vel, acc, pos_dot3, vel_dot3);
        next_pos = pos + pos_dot3 * dt;
        next_vel = vel + vel_dot3 * dt;
        deq(next_pos, next_vel, acc, pos_dot4, vel_dot4);
        pos += (pos_dot1 + 2.0 * pos_dot2 + 2.0 * pos_dot3 + pos_dot4) * dt / 6.0;
        vel += (vel_dot1 + 2.0 * vel_dot2 + 2.0 * vel_dot3 + vel_dot4) * dt / 6.0;
    }

    double geph2svdt(const gtime_t &curr_time, const GloEphemPtr geph_ptr)
    {
        double dt = time_diff(curr_time, geph_ptr->toe);
        if (dt > WEEK_SECONDS/2)
            dt -= WEEK_SECONDS;
        else if (dt < -WEEK_SECONDS/2)
            dt += WEEK_SECONDS;
        
        for (size_t i = 0; i < 2; ++i)
            dt -= -geph_ptr->tau_n + geph_ptr->gamma * dt;
        double svdt = -geph_ptr->tau_n + geph_ptr->gamma * dt;
        return svdt;
    }

    Eigen::Vector3d geph2pos(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *svdt)
    {
        Eigen::Vector3d sv_pos, sv_vel, sv_acc;
        sv_pos << geph_ptr->pos[0], geph_ptr->pos[1], geph_ptr->pos[2];
        sv_vel << geph_ptr->vel[0], geph_ptr->vel[1], geph_ptr->vel[2];
        sv_acc << geph_ptr->acc[0], geph_ptr->acc[1], geph_ptr->acc[2];
        
        double dt = time_diff(curr_time, geph_ptr->toe);
        double dts = -geph_ptr->tau_n + geph_ptr->gamma * dt;
        if (svdt)   *svdt = dts;
        
        for (double tt = dt<0.0?-TSTEP:TSTEP; fabs(dt) > 1e-9; dt -= tt) {
            if (fabs(dt) < TSTEP) tt = dt;
            glo_orbit(tt, sv_pos, sv_vel, sv_acc);
        }
        return sv_pos;
    }

    Eigen::Vector3d geph2vel(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *svddt)
    {
        Eigen::Vector3d sv_pos, sv_vel, sv_acc;
        sv_pos << geph_ptr->pos[0], geph_ptr->pos[1], geph_ptr->pos[2];
        sv_vel << geph_ptr->vel[0], geph_ptr->vel[1], geph_ptr->vel[2];
        sv_acc << geph_ptr->acc[0], geph_ptr->acc[1], geph_ptr->acc[2];
        
        double dt = time_diff(curr_time, geph_ptr->toe);
        if (svddt)   *svddt = geph_ptr->gamma;
        
        for (double tt = dt<0.0?-TSTEP:TSTEP; fabs(dt) > 1e-9; dt -= tt) {
            if (fabs(dt) < TSTEP) tt = dt;
            glo_orbit(tt, sv_pos, sv_vel, sv_acc);
        }
        return sv_vel;
    }

    Eigen::Vector3d ecef2enu(const Eigen::Vector3d &ref_lla, const Eigen::Vector3d &v_ecef)
    {
        double lat = ref_lla.x() * D2R, lon = ref_lla.y() * D2R;
        double sin_lat = sin(lat), cos_lat = cos(lat);
        double sin_lon = sin(lon), cos_lon = cos(lon);
        Eigen::Matrix3d R_enu_ecef;
        R_enu_ecef << -sin_lon,             cos_lon,         0,
                    -sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat,
                     cos_lat*cos_lon,  cos_lat*sin_lon, sin_lat;
        return (R_enu_ecef * v_ecef);
    }

    Eigen::Matrix3d geo2rotation(const Eigen::Vector3d &ref_geo)
    {
        double lat = ref_geo.x() * D2R, lon = ref_geo.y() * D2R;
        double sin_lat = sin(lat), cos_lat = cos(lat);
        double sin_lon = sin(lon), cos_lon = cos(lon);
        Eigen::Matrix3d R_ecef_enu;
        R_ecef_enu << -sin_lon, -sin_lat*cos_lon, cos_lat*cos_lon,
                       cos_lon, -sin_lat*sin_lon, cos_lat*sin_lon,
                       0      ,  cos_lat        , sin_lat;
        return R_ecef_enu;
    }

    Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d &ref_ecef)
    {
        return geo2rotation(ecef2geo(ref_ecef));
    }

    void sat_azel(const Eigen::Vector3d &rev_pos, const Eigen::Vector3d &sat_pos, double *azel)
    {
        if (!azel)  return;
        Eigen::Vector3d rev_lla = ecef2geo(rev_pos);
        Eigen::Vector3d rev2sat_ecef = (sat_pos - rev_pos).normalized();
        Eigen::Vector3d rev2sat_enu = ecef2enu(rev_lla, rev2sat_ecef);
        azel[0] = rev2sat_ecef.head<2>().norm() < 1e-12 ? 0.0 : atan2(rev2sat_enu.x(), rev2sat_enu.y());
        azel[0] += (azel[0] < 0 ? 2*M_PI : 0);
        azel[1] = asin(rev2sat_enu.z());
    }
    
    /* latitude in degree */
    static double interpc(const double coef[], double lat)
    {
        int i=(int)(lat/15.0);
        if (i<1) return coef[0]; else if (i>4) return coef[4];
        return coef[i-1]*(1.0-lat/15.0+i)+coef[i]*(lat/15.0-i);
    }

    /* elevation in radius */
    static double mapf(double el, double a, double b, double c)
    {
        double sinel=sin(el);
        return (1.0+a/(1.0+b/(1.0+c)))/(sinel+(a/(sinel+b/(sinel+c))));
    }

    /* calculate troposphere hydrostatic/wet mapping functions  ------------------------------
    * calculate troposphere hydrostatic/wet mapping functions by Niell model
    * ref: Global mapping functions for the atmosphere delay at radio wavelengths
    * args   : gtime_t time                 I   time (gpst)
    * args   : Eigen::Vector3d rev_lla      I   receiver position in geodetic coordinate
    *          double*         azel         I   satellite azimuth/elevation angle in radius
    *          double*         mapfh        IO  hydrostatic mapping coefficient
    * return : delay expressed by light travel distance (m)
    *-----------------------------------------------------------------------------*/
    static void nmf(gtime_t time, const Eigen::Vector3d rev_lla, const double azel[], 
                    double *mapfh, double *mapfw)
    {
        /* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
        const double coef[][5]={
            { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
            { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
            { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
            
            { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
            { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
            { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
            
            { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
            { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
            { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
        };
        const double aht[]={ 2.53E-5, 5.49E-3, 1.14E-3}; /* height correction */
        
        double ah[3],aw[3],dm,el=azel[1],lat=rev_lla.x(),hgt=rev_lla.z();
        int i;
        
        if (el<=0.0) {
            if (mapfw) *mapfw=0.0;
            if (mapfh) *mapfh=0.0;
            return;
        }
        /* year from doy 28, added half a year for southern latitudes */
        double y=(time2doy(time)-28.0)/365.25+(lat<0.0?0.5:0.0);
        
        double cosy=cos(2.0*M_PI*y);
        lat=fabs(lat);
        
        for (i=0;i<3;i++) {
            ah[i]=interpc(coef[i  ],lat)-interpc(coef[i+3],lat)*cosy;
            aw[i]=interpc(coef[i+6],lat);
        }
        /* ellipsoidal height is used instead of height above sea level */
        dm=(1.0/sin(el)-mapf(el,aht[0],aht[1],aht[2]))*hgt/1E3;
        
        if (mapfw) *mapfw=mapf(el,aw[0],aw[1],aw[2]);
        if (mapfh) *mapfh=mapf(el,ah[0],ah[1],ah[2])+dm;
    }

    double calculate_trop_delay(gtime_t time, const Eigen::Vector3d &rev_lla, const double *azel)
    {
        // compute tropospheric delay by standard atmosphere and saastamoinen model
        const double temp0=15.0; /* temparature at sea level */
        const double humi =0.7;  /* relative humidity */ 
        
        if (rev_lla.z()<-100.0||1E4<rev_lla.z()||azel[1]<=0) return 0.0;
        
        /* standard atmosphere */
        double hgt=rev_lla.z()<0.0?0.0:rev_lla.z();
        
        double pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
        double temp=temp0-6.5E-3*hgt+273.16;
        double e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));
        
        /* saastamoninen model */
        double zhd=0.0022768*pres/(1.0-0.00266*cos(2.0*rev_lla.x()*D2R)-0.00028*hgt/1E3);
        double zwd=0.002277*(1255.0/temp+0.05)*e;

        double mapfh = 0.0, mapfw = 0.0;
        nmf(time, rev_lla, azel, &mapfh, &mapfw);
        return (mapfh*zhd + mapfw*zwd);
    }

    double calculate_ion_delay(gtime_t t, const std::vector<double> &ion_parameters, 
                               const Eigen::Vector3d &rev_lla, const double *azel)
    {
        if (ion_parameters.empty())         return 0.0;
        if (rev_lla.z()<-1E3||azel[1]<=0)   return 0.0;
        
        /* earth centered angle (semi-circle) */
        double psi=0.0137/(azel[1]/M_PI+0.11)-0.022;
        
        /* subionospheric latitude/longitude (semi-circle) */
        double phi=rev_lla.x()/180.0+psi*cos(azel[0]);
        if      (phi> 0.416) phi= 0.416;
        else if (phi<-0.416) phi=-0.416;
        double lam=rev_lla.y()/180.0+psi*sin(azel[0])/cos(phi*M_PI);
        
        /* geomagnetic latitude (semi-circle) */
        phi+=0.064*cos((lam-1.617)*M_PI);
        
        /* local time (s) */
        uint32_t week = 0;
        double tt=43200.0*lam+time2gpst(t,&week);
        tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */
        
        /* slant factor */
        double f=1.0+16.0*pow(0.53-azel[1]/M_PI,3.0);
        
        /* ionospheric delay */
        double amp=ion_parameters[0]+phi*(ion_parameters[1]+phi*(ion_parameters[2]+phi*ion_parameters[3]));
        double per=ion_parameters[4]+phi*(ion_parameters[5]+phi*(ion_parameters[6]+phi*ion_parameters[7]));
        amp=amp<    0.0?    0.0:amp;
        per=per<72000.0?72000.0:per;
        double x=2.0*M_PI*(tt-50400.0)/per;
        
        return LIGHT_SPEED*f*(fabs(x)<1.57?5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)):5E-9);
    }

    std::string sat2str(uint32_t sat_no)
    {
        std::stringstream ss;
        uint32_t prn = 0;
        uint32_t sys = satsys(sat_no, &prn);
        switch (sys)
        {
            case SYS_GPS:
                ss << "G" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_GLO:
                ss << "R" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_BDS:
                ss << "C" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_GAL:
                ss << "E" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_SBS:
                ss << "S" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_QZS:
                ss << "J" << std::setw(2) << std::setfill('0') << prn;
                break;
            default:    // current not support other system
                LOG(WARNING) << "currently not support satelite system id " << sys;
                break;
        }
        return ss.str();
    }

    double L1_freq(const ObsPtr &obs, int *l1_idx)
    {
        const uint32_t sys = satsys(obs->sat, NULL);
        double freq_min = -1.0;
        double freq_max = -1.0;
        if (sys == SYS_GPS || sys == SYS_GAL)
            freq_min = freq_max = FREQ1;
        else if (sys == SYS_BDS)
            freq_min = freq_max = FREQ1_BDS;
        else if (sys == SYS_GLO)
        {
            freq_min = FREQ1_GLO - 7 * DFRQ1_GLO;
            freq_max = FREQ1_GLO + 6 * DFRQ1_GLO;
        }
        
        if (l1_idx != NULL)
            *l1_idx = -1;
        
        for (uint32_t i = 0; i < obs->freqs.size(); ++i)
        {
            if (obs->freqs[i] >= freq_min && obs->freqs[i] <= freq_max)
            {
                if (l1_idx != NULL)
                    *l1_idx = static_cast<int>(i);
                return obs->freqs[i];
            }
        }

        return -1.0;
    }

    uint32_t str2sat(const std::string &sat_str)
    {
        if (sat_str.size() < 3)
        {
            LOG(ERROR) << "Invalid RINEX satellite identifier " << sat_str;
            return 0;
        }
        const uint32_t prn = std::stoul(sat_str.substr(1, 2));
        switch (sat_str.at(0))
        {
            case 'G':
                return sat_no(SYS_GPS, prn);
            case 'R':
                return sat_no(SYS_GLO, prn);
            case 'C':
                return sat_no(SYS_BDS, prn);
            case 'E':
                return sat_no(SYS_GAL, prn);
            case 'S':
                return sat_no(SYS_SBS, prn);
            case 'J':
                return sat_no(SYS_QZS, prn);
        }
        return 0;
    }

    std::string exec(const std::string &cmd) 
    {
        char buffer[128];
        std::string result = "";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) throw std::runtime_error("popen() failed!");
        try 
        {
            while (fgets(buffer, sizeof buffer, pipe) != NULL) 
            {
                result += buffer;
            }
        } 
        catch (...) 
        {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;
    }

}   // namespace gnss_comm
