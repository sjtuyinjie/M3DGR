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

#ifndef RINEX_HELPER_HPP_
#define RINEX_HELPER_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <map>
#include <time.h>
#include <glog/logging.h>

#include "gnss_constant.hpp"
#include "gnss_utility.hpp"

namespace gnss_comm
{
    /* parse ephemeris from RINEX navigation file  ----------------------------------------------------------
    * args   : std::string                                  rinex_filepath        I   RINEX file path
    *          std::map<uint32_t, std::vecot<EphemBase>>&   sat2ephem             IO  satellite number to ephemeris
    * return : None
    *---------------------------------------------------------------------------------------------*/
    void rinex2ephems(const std::string &rinex_filepath, 
        std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem);
    
    /* parse GNSS measurement from RINEX observation file  ---------------------------------------
    * args   : std::string                          rinex_filepath        I   RINEX file path
    *          std::vector<std::vector<ObsPtr>>&    rinex_meas            IO  GNSS measurement in time order
    * return : None
    *---------------------------------------------------------------------------------------------*/
    void rinex2obs(const std::string &rinex_filepath, 
        std::vector<std::vector<ObsPtr>> &rinex_meas);
    
    /* output GNSS measurement to RINEX file  -----------------------------------------------------
    * args   : std::string                          rinex_filepath        I   RINEX file path
    *          std::vector<std::vector<ObsPtr>>&    rinex_meas            I   GNSS measurement in time order
    * return : None
    * This function is still under experiment, many hacks here.
    *---------------------------------------------------------------------------------------------*/
    void obs2rinex(const std::string &rinex_filepath, 
        const std::vector<std::vector<ObsPtr>> &gnss_meas);

    /* parse GPS ionosphere from RINEX navigation file  ------------------------------------------
    * args   : std::string                      rinex_filepath        I   RINEX file path
    *          std::vector<double>              iono_params           IO  8 ionosphere parameters (alpha 1~4, beta 1~4)
    * return : None
    *---------------------------------------------------------------------------------------------*/
    void rinex2iono_params(const std::string &rinex_filepath, std::vector<double> &iono_params);


}   // namespace gnss_comm


#endif