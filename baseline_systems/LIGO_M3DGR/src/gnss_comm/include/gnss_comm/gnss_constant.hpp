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
* As most of the GNSS-related constants are adapted from RTKLIB, 
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

#ifndef GNSS_CONSTANT_HPP_
#define GNSS_CONSTANT_HPP_

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <memory>
#include <iomanip>
#include <eigen3/Eigen/Dense>

namespace gnss_comm
{
    // some parameters are adapted from RTKLIB
    #define MAXFREQ     7                   /* max N_FREQ */

    #define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
    #define FREQ2       1.22760E9           /* L2     frequency (Hz) */
    #define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
    #define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
    #define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
    #define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
    #define FREQ9       2.492028E9          /* S      frequency (Hz) */
    #define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
    #define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
    #define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
    #define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
    #define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
    #define FREQ1_BDS   1.561098E9          /* BeiDou B1 frequency (Hz) */
    #define FREQ2_BDS   1.20714E9           /* BeiDou B2 frequency (Hz) */
    #define FREQ3_BDS   1.26852E9           /* BeiDou B3 frequency (Hz) */

    #define SYS_NONE    0x00                /* navigation system: none */
    #define SYS_GPS     0x01                /* navigation system: GPS */
    #define SYS_SBS     0x02                /* navigation system: SBAS */
    #define SYS_GLO     0x04                /* navigation system: GLONASS */
    #define SYS_GAL     0x08                /* navigation system: Galileo */
    #define SYS_QZS     0x10                /* navigation system: QZSS */
    #define SYS_BDS     0x20                /* navigation system: BeiDou */
    #define SYS_IRN     0x40                /* navigation system: IRNSS */
    #define SYS_LEO     0x80                /* navigation system: LEO */
    #define SYS_ALL     0xFF                /* navigation system: all */

    #define T_SYS_GPS       0               /* time system: GPS time */
    #define T_SYS_UTC       1               /* time system: UTC */
    #define T_SYS_GLO       2               /* time system: GLONASS time */
    #define T_SYS_GAL       3               /* time system: Galileo time */
    #define T_SYS_QZS       4               /* time system: QZSS time */
    #define T_SYS_BDS       5               /* time system: BeiDou time */

    #ifndef N_FREQ
    #define N_FREQ          3                   /* number of carrier frequencies */
    #endif
    #define N_FREQ_GLO      2                   /* number of carrier frequencies of GLONASS */

    #define MIN_PRN_GPS     1                   /* min satellite PRN number of GPS */
    #define MAX_PRN_GPS     32                  /* max satellite PRN number of GPS */
    #define N_SAT_GPS       (MAX_PRN_GPS-MIN_PRN_GPS+1) /* number of GPS satellites */
    #define N_SYS_GPS       1

    // #define MIN_PRN_GLO     38                   /* min satellite slot number of GLONASS */ For UM4B0
    // #define MAX_PRN_GLO     61                  /* max satellite slot number of GLONASS */  For UM4B0
    #define MIN_PRN_GLO     1                   /* min satellite slot number of GLONASS */
    #define MAX_PRN_GLO     27                  /* max satellite slot number of GLONASS */
    #define N_SAT_GLO       (MAX_PRN_GLO-MIN_PRN_GLO+1) /* number of GLONASS satellites */
    #define N_SYS_GLO       1

    #define MIN_PRN_GAL     1                   /* min satellite PRN number of Galileo */
    #define MAX_PRN_GAL     38                  /* max satellite PRN number of Galileo */
    #define N_SAT_GAL       (MAX_PRN_GAL-MIN_PRN_GAL+1) /* number of Galileo satellites */
    #define N_SYS_GAL       1

    #define MIN_PRN_BDS     1                   /* min satellite sat number of BeiDou */
    #define MAX_PRN_BDS     63                  /* max satellite sat number of BeiDou */
    #define N_SAT_BDS       (MAX_PRN_BDS-MIN_PRN_BDS+1) /* number of BeiDou satellites */
    #define N_SYS_BDS       1

    #define N_SYS           (N_SYS_GPS+N_SYS_GLO+N_SYS_GAL++N_SYS_BDS) /* number of systems */

    #define MAX_SAT         (N_SAT_GPS+N_SAT_GLO+N_SAT_GAL+N_SAT_BDS)

    #ifndef MAXOBS
    #define MAXOBS      64                  /* max number of obs in an epoch */
    #endif

    // solution type
    // #define SOL_COMPUTED            0
    // #define SOL_INSUFFICIENT_OBS    1
    // #define NO_CONVERGENCE          2
    // #define COV_TRACE               4

    // extend solution type
    // #define EXT_NONE                0x00
    // #define EXT_RTK                 0x01
    // #define EXT_IONO_BRDC           0x02
    // #define EXT_IONO_SBAS           0x04
    // #define EXT_IONO_IFLC           0x06
    // #define EXT_IONO_DIFF           0x08

    // position(velocity) type
    #define PV_NONE                     0               /* no solution */
    #define PV_FIXED_POS                1
    #define PV_FIXED_HEIGHT             2
    #define PV_DOPPLER_VELOCITY         8
    #define PV_SINGLE                   16
    #define PV_PSR_DIFF                 17
    #define PV_WASS                     18
    #define PV_L1_FLOAT                 32
    #define PV_IONO_FREE_FLOAT          33
    #define PV_NARROW_FLOW              34
    #define PV_L1_INT                   48
    #define PV_WIDE_INT                 49
    #define PV_NARROW_INT               50
    #define PV_INS                      52
    #define PV_INS_PSR_SP               53
    #define PV_INS_PSR_DIFF             54
    #define PV_INS_RTK_FLOAT            55
    #define PV_INS_RTK_FIXED            56

    #define CODE_NONE   0                   /* obs code: none or unknown */
    #define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
    #define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
    #define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
    #define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
    #define CODE_L1M    5                   /* obs code: L1M        (GPS) */
    #define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
    #define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
    #define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
    #define CODE_L1E    9                   /* obs code: L1-SAIF    (QZS) */
    #define CODE_L1A    10                  /* obs code: E1A        (GAL) */
    #define CODE_L1B    11                  /* obs code: E1B        (GAL) */
    #define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
    #define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
    #define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
    #define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
    #define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
    #define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
    #define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
    #define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
    #define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
    #define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
    #define CODE_L2M    22                  /* obs code: L2M        (GPS) */
    #define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
    #define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
    #define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
    #define CODE_L5X    26                  /* obs code: L5/E5aI+Q  (GPS,GAL,QZS,SBS) */
    #define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
    #define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
    #define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
    #define CODE_L6A    30                  /* obs code: E6A        (GAL) */
    #define CODE_L6B    31                  /* obs code: E6B        (GAL) */
    #define CODE_L6C    32                  /* obs code: E6C        (GAL) */
    #define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
    #define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
    #define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
    #define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
    #define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
    #define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
    #define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
    #define CODE_L2I    40                  /* obs code: B1I        (CMP) */
    #define CODE_L2Q    41                  /* obs code: B1Q        (CMP) */
    #define CODE_L6I    42                  /* obs code: B3I        (CMP) */
    #define CODE_L6Q    43                  /* obs code: B3Q        (CMP) */
    #define CODE_L3I    44                  /* obs code: G3I        (GLO) */
    #define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
    #define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
    #define CODE_L1I    47                  /* obs code: B1I        (BDS) */
    #define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
    #define MAXCODE     48                  /* max number of obs code */

    #define EARTH_ECCE_2            6.69437999014e-3    // WGS 84 (Earth eccentricity)^2 (m^2)
    #define EARTH_MEAN_RADIUS       6371009             // Mean R of ellipsoid(m) IU Gedosey& Geophysics
    #define EARTH_SEMI_MAJOR        6378137             // WGS 84 Earth semi-major axis (m)
    #define EARTH_SEMI_MAJOR_GLO    6378136.0           // radius of earth (m)
    #define EARTH_OMG_GLO           7.2921150000e-5     // GLO value of earth's rotation rate (rad/s)
    #define EARTH_OMG_GPS           7.2921151467e-5     // GPS/GAL value of earth's rotation rate (rad/s)
    #define EARTH_OMG_BDS           7.2921150000e-5     // BDS value of earth's rotation rate (rad/s)
    #define MU_GPS                  3.9860050000e14     // gravitational constant (GPS)      
    #define MU                      3.9860044180e14     // gravitational constant (GAL, BDS, GLO)    
    #define TSTEP                   60.0                // integration step glonass ephemeris (s)
    #define J2_GLO                  1.0826257E-3        // 2nd zonal harmonic of geopot
    #define LIGHT_SPEED             2.99792458e8        // WGS-84 Speed of light in a vacuum (m/s)
    #define GPS_EPHCH_JD            2444244.5           // GPS Epoch in Julian Days
    #define EPH_VALID_SECONDS       7200                // 2 hours ephemeris validity
    #define WEEK_SECONDS            604800              // Seconds within one week
    #define EPSILON_KEPLER          1e-14               // Kepler equation terminate condition
    #define MAX_ITER_KEPLER         30                  // Kepler equation maximum iteration number
    #define EPSILON_PVT             1e-8                // PVT terminate condition
    #define MAX_ITER_PVT            30                  // PVT maximum iteration number
    #define RANGE_FREQ              1                   // Range measurement frequency
    #define R2D                     (180.0/M_PI)        // radius to degree
    #define D2R                     (M_PI/180.0)        // degree to radius
    #define SC2RAD                  3.1415926535898     /* semi-circle to radian (IS-GPS) */
    #define SIN_N5                  -0.0871557427476582 // sin(-5.0 deg)
    #define COS_N5                   0.9961946980917456 // cos(-5.0 deg)

    #define INVALID_CLK             999999.999999

    #define MAXLEAPS                64
    const double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
        {2017,1,1,0,0,0,-18},
        {2015,7,1,0,0,0,-17},
        {2012,7,1,0,0,0,-16},
        {2009,1,1,0,0,0,-15},
        {2006,1,1,0,0,0,-14},
        {1999,1,1,0,0,0,-13},
        {1997,7,1,0,0,0,-12},
        {1996,1,1,0,0,0,-11},
        {1994,7,1,0,0,0,-10},
        {1993,7,1,0,0,0, -9},
        {1992,7,1,0,0,0, -8},
        {1991,1,1,0,0,0, -7},
        {1990,1,1,0,0,0, -6},
        {1988,1,1,0,0,0, -5},
        {1985,7,1,0,0,0, -4},
        {1983,7,1,0,0,0, -3},
        {1982,7,1,0,0,0, -2},
        {1981,7,1,0,0,0, -1},
        {0}
    };

    /* System identifier to system code */
    const std::map<uint8_t, uint32_t> char2sys = 
    {
        {'G', SYS_GPS},
        {'C', SYS_BDS},
        {'R', SYS_GLO},
        {'E', SYS_GAL}
    };

    /* System map to index */
    const std::map<uint32_t, uint32_t> sys2idx = 
    {
        {SYS_GPS, 0},
        {SYS_GLO, 1},
        {SYS_GAL, 2},
        {SYS_BDS, 3}
    };

    /* RINEX frequency encoding to frequency value */
    const std::map<std::string, double> type2freq = 
    {
        {"G1", FREQ1},
        {"G2", FREQ2},
        {"G5", FREQ5},
        {"R1", FREQ1_GLO},
        {"R2", FREQ2_GLO},
        {"R3", 1.202025E9},
        {"R4", 1.600995E9},
        {"R6", 1.248060E9},
        {"E1", FREQ1},
        {"E5", FREQ5},
        {"E6", FREQ6},
        {"E7", FREQ7},
        {"E8", FREQ8},
        {"C1", FREQ1},
        {"C2", FREQ1_BDS},
        {"C5", FREQ5},
        {"C6", FREQ3_BDS},
        {"C7", FREQ2_BDS},
        {"C8", FREQ8}
    };

    struct gtime_t
    {
        time_t time;            /* time (s) expressed by standard time_t */
        double sec;             /* fraction of second under 1 s */
    };

    struct EphemBase
    {
        virtual ~EphemBase() = default;
        uint32_t sat;                   /* satellite number */
        gtime_t  ttr;                   /* transmission time in GPST */
        gtime_t  toe;                   /* ephemeris reference time in GPST */
        uint32_t health;                /* satellite health */
        double   ura;                   /* satellite signal accuracy */
        uint32_t iode;
    };
    typedef std::shared_ptr<EphemBase> EphemBasePtr;

    struct GloEphem : EphemBase
    {
        int         freqo;              /* satellite frequency number */
        uint32_t    age;                /* satellite age */
        double      pos[3];             /* satellite position (ecef) (m) */
        double      vel[3];             /* satellite velocity (ecef) (m/s) */
        double      acc[3];             /* satellite acceleration (ecef) (m/s^2) */
        double      tau_n, gamma;       /* SV clock bias (s)/relative freq bias */
        double      delta_tau_n;        /* delay between L1 and L2 (s) */
    };
    typedef std::shared_ptr<GloEphem> GloEphemPtr;

    struct Ephem : EphemBase
    {
        gtime_t     toc;                    /* clock correction reference time in GPST */
        double      toe_tow;                /* toe seconds within the week */
        uint32_t    week;
        uint32_t    iodc;
        uint32_t    code;
        double      A, e, i0, omg, OMG0, M0, delta_n, OMG_dot, i_dot;       /* SV orbit parameters */
        double      cuc, cus, crc, crs, cic, cis;
        double      af0, af1, af2;          /* SV clock parameters */
        double      tgd[2];                 /* group delay parameters */
                                            /* GPS    :tgd[0]=TGD */
                                            /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                                            /* BDS    :tgd[0]=BGD1,tgd[1]=BGD2 */
        double A_dot, n_dot;                /* Adot,ndot for CNAV */
    };
    typedef std::shared_ptr<Ephem> EphemPtr;

    struct Obs                                           /* observation data record */
    {
        gtime_t     time;                                /* receiver sampling time (GPST) */
        uint32_t    sat;                                 /* satellite number */
        std::vector<double> freqs;                       /* received satellite frequencies */
        std::vector<double> CN0;                         /* signal strength */
        std::vector<uint8_t> LLI;                        /* signal lost-lock indictor */
        std::vector<uint8_t> code;                       /* code indicator (CODE_???) */
        std::vector<double> psr;                         /* observation data pseudorange (m) */
        std::vector<double> psr_std;                     /* pseudorange std (m) */
        std::vector<double> cp;                          /* observation data carrier-phase (cycle) */
        std::vector<double> cp_std;                      /* carrier-phase std (cycle) */
        std::vector<double> dopp;                        /* observation data doppler frequency (Hz) */
        std::vector<double> dopp_std;                    /* doppler std (Hz) */
        std::vector<uint8_t> status;                     /* cycle slip valid flag. bit_0 (psr valid), bit_1(cp valid), bit_2(half cp valid, *ublox), bit_3(half cp subtracted, *ublox) */
    };
    typedef std::shared_ptr<Obs> ObsPtr;

    struct TimePulseInfo                /* time pulse(PPS) information */
    {
        gtime_t time;
        bool utc_based;
        uint32_t time_sys;
    };
    typedef std::shared_ptr<TimePulseInfo> TimePulseInfoPtr;

    struct BestSat
    {
        gtime_t     time;
        uint32_t    sat;
        uint8_t     status;
        std::set<double> freq_used;      /* frequency used in solution */
    };
    typedef std::shared_ptr<BestSat> BestSatPtr;

    struct BestPos
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    pos_type;
        double      lat, lon, hgt;
        double      lat_sigma, lon_sigma, hgt_sigma;
        double      undulation;
        double      diff_age;
        double      sol_age;
        uint8_t     num_svs;
        uint8_t     num_soln_svs;
        uint8_t     ext_sol_stat;
        uint8_t     sig_mask;
    };
    typedef std::shared_ptr<BestPos> BestPosPtr;

    struct BestVel
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    vel_type;
        double      latency;
        double      age;
        double      hor_speed, vel_speed;
        double      direction2north_degree;
    };
    typedef std::shared_ptr<BestVel> BestVelPtr;

    struct BestXYZ
    {
        gtime_t     time;
        uint32_t    pos_sol_status;
        uint32_t    pos_type;
        double      pos[3], pos_sigma[3];
        uint32_t    vel_sol_status;
        uint32_t    vel_type;
        double      vel[3], vel_sigma[3];
        double      vel_latency;
        double      diff_age;
        double      sol_age;
        uint8_t     num_svs;
        uint8_t     num_soln_svs;
        uint8_t     ext_sol_stat;
        uint8_t     sig_mask;
    };
    typedef std::shared_ptr<BestXYZ> BestXYZPtr;

    struct PVTSolution
    {
        gtime_t time;
        uint8_t fix_type;   // 0-no fix, 1-dead reckoning only, 2-2D, 3-3D, 4-GNSS+(1), 5-time only fix 
        bool valid_fix;
        bool diff_soln;     // whether differential applied
        uint8_t carr_soln;  // 0-no cp solution, 1-float, 2-fixed
        uint8_t num_sv;
        double lat, lon, hgt;
        double hgt_msl;     // height above mean sea level
        double h_acc, v_acc; // horizontal and vertical accuracy
        double p_dop;       // position DOP
        double vel_n, vel_e, vel_d; // velocity in NED frame
        double vel_acc;         // velocity accuracy
    };
    typedef std::shared_ptr<PVTSolution> PVTSolutionPtr;

    struct PsrPos
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    pos_type;
        double      lat, lon, hgt;
        double      lat_sigma, lon_sigma, hgt_sigma;
        double      undulation;
        double      diff_age;
        double      sol_age;
        uint8_t     num_svs;
        uint8_t     num_soln_svs;
        uint8_t     ext_sol_stat;
        uint8_t     sig_mask;
    };
    typedef std::shared_ptr<PsrPos> PsrPosPtr;

    struct PsrVel
    {
        gtime_t     time;
        uint32_t    sol_status;
        uint32_t    vel_type;
        double      latency;
        double      age;                /* differential age */
        double      hor_spd;
        double      trk_gnd;
        double      vert_spd;
    };
    typedef std::shared_ptr<PsrVel> PsrVelPtr;

    struct SvInfo
    {
        gtime_t     time;
        uint32_t    sat;
        uint32_t    freqo;
        bool        health;
        double      elev_degree;
        double      az_degree;
        uint32_t    sig_mask;
    };
    typedef std::shared_ptr<SvInfo> SvInfoPtr;

    struct TECEpoch
    {
        gtime_t time;       // epoch time
        double rb;          // earth radius (km)
        double lla_start[3];
        double lla_end[3];
        double lla_interval[3];
        std::vector<double> data;  // num_hgt * num_lat * num_lon
        std::vector<double> rms;   // num_hgt * num_lat * num_lon
    };
    typedef std::shared_ptr<TECEpoch> TECEpochPtr;

    struct EphemerideEpoch
    {
        gtime_t time;
        std::map<uint32_t, Eigen::Vector3d> sat2pos;
        std::map<uint32_t, double> sat2clk;
    };
    typedef std::shared_ptr<EphemerideEpoch> EphemerideEpochPtr;

    struct SatState
    {
        uint32_t sat_id;
        gtime_t  ttx;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double dt;
        double ddt;
        double tgd;
    };
    typedef std::shared_ptr<SatState> SatStatePtr;
}   // namespace gnss_comm

#endif




