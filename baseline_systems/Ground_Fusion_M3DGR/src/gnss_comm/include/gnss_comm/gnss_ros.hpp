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
*/

#ifndef GNSS_ROS_HPP_
#define GNSS_ROS_HPP_

#include <ros/ros.h>
#include <gnss_comm/GnssTimeMsg.h>
#include <gnss_comm/GnssEphemMsg.h>
#include <gnss_comm/GnssGloEphemMsg.h>
#include <gnss_comm/GnssObsMsg.h>
#include <gnss_comm/GnssMeasMsg.h>
#include <gnss_comm/GnssBestXYZMsg.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <gnss_comm/GnssSvsMsg.h>
#include <gnss_comm/GnssTimePulseInfoMsg.h>
#include <gnss_comm/StampedFloat64Array.h>

#include "gnss_constant.hpp"
#include "gnss_utility.hpp"

namespace gnss_comm
{
    /* convert Ephem struct to ros message ---------------------------------------
    * args   : Ephem &  ephem      I   Ephemeris
    * return : cooresponding ephemeris message
    *-----------------------------------------------------------------------------*/
    GnssEphemMsg ephem2msg(const EphemPtr &ephem_ptr);

    /* parse Ephem struct from ros message ----------------------------------------------
    * args   : GnssEphemConstPtr &  gnss_ephem_msg  I   Ephemeris ros message
    * return : cooresponding ephemeris struct
    *-----------------------------------------------------------------------------------*/
    EphemPtr msg2ephem(const GnssEphemMsgConstPtr &gnss_ephem_msg);

    /* convert GloEphem struct to ros message ---------------------------------------
    * args   : GloEphem &  glo_ephem      I   GLONASS Ephemeris
    * return : cooresponding ephemeris message
    *-----------------------------------------------------------------------------*/
    GnssGloEphemMsg glo_ephem2msg(const GloEphemPtr &glo_ephem_ptr);

    /* parse GloEphem struct from ros message ----------------------------------------------
    * args   : GnssGloEphemConstPtr & gnss_glo_ephem_msg  I   GLONASS Ephemeris ros message
    * return : cooresponding GLONASS ephemeris struct
    *--------------------------------------------------------------------------------------*/
    GloEphemPtr msg2glo_ephem(const GnssGloEphemMsgConstPtr &gnss_glo_ephem_msg);

    /* convert GNSS measurements to ros message ----------------------------------
    * args   : std::vector<ObsPtr> & meas      I   GNSS measurements
    * return : cooresponding GNSS measurement message
    *-----------------------------------------------------------------------------*/
    GnssMeasMsg meas2msg(const std::vector<ObsPtr> &meas);

    /* parse GNSS measurements from ros message ----------------------------------
    * args   : GnssMeasConstPtr & gnss_meas_msg      I   GNSS measurement message
    * return : cooresponding GNSS measurements
    *-----------------------------------------------------------------------------*/
    std::vector<ObsPtr> msg2meas(const GnssMeasMsgConstPtr &gnss_meas_msg);

    /* convert GNSS time pulse information to ros message ----------------------------------
    * args   : TimePulseInfoPtr tp_info      I   GNSS time pulse information
    * return : cooresponding GNSS time pulse information message
    *-----------------------------------------------------------------------------*/
    GnssTimePulseInfoMsg tp_info2msg(const TimePulseInfoPtr &tp_info);

    /* parse GNSS time pulse information from ros message ---------------------------------
    * args   : GnssTimePulseInfoMsgConstPtr gnss_tp_info_msg   I   GNSS time pulse message
    * return : cooresponding GNSS time pulse information
    *-----------------------------------------------------------------------------*/
    TimePulseInfoPtr msg2tp_info(const GnssTimePulseInfoMsgConstPtr &gnss_tp_info_msg);

    /* convert GNSS best estimated xyz to ros message ----------------------------
    * args   : BestXYZ & best_xyz      I   GNSS estimated xyz in ECEF
    * return : cooresponding GNSS best xyz message
    *-----------------------------------------------------------------------------*/
    GnssBestXYZMsg best_xyz2msg(const BestXYZPtr &best_xyz);

    /* convert GNSS PVT solution to ros message -----------------------------------
    * args   : PVTSolutionPtr & pvt_soln      I   GNSS PVT solution
    * return : cooresponding GNSS PVT solution message
    *-----------------------------------------------------------------------------*/
    GnssPVTSolnMsg pvt2msg(const PVTSolutionPtr &pvt_soln);

    /* parse GNSS PVT solution from ros message ---------------------------------
    * args   : GnssPVTSolnMsgConstPtr pvt_msg   I   GNSS PVT solution message
    * return : cooresponding GNSS PVT solution
    *-----------------------------------------------------------------------------*/
    PVTSolutionPtr msg2pvt(const GnssPVTSolnMsgConstPtr &pvt_msg);

    /* convert GNSS space vehicles' info to ros message ----------------------------
    * args   : std::vector<SvInfo> & svs      I   GNSS space vehicles' info
    * return : cooresponding GNSS space vehicle info message
    *-----------------------------------------------------------------------------*/
    GnssSvsMsg svs2msg(const std::vector<SvInfo> &svs);

}   // namespace gnss_comm

#endif