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

#include "GNSS_Assignment.h"

GNSSAssignment::GNSSAssignment() : process_feat_num(0) {
    // fout_std.open((string(string(ROOT_DIR) + "Log/"+ "std.txt")), ios::out);
}

void GNSSAssignment::initNoises( void ) // maybe usable!
{
    gtsam::Vector priorrotNoiseVector3(3);
    priorrotNoiseVector3 << marg_noise, marg_noise, marg_noise; // prior_noise / 100, prior_noise / 100, prior_noise / 100;
    // priorrotNoiseVector3 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000;
    priorrotNoise = gtsam::noiseModel::Diagonal::Variances(priorrotNoiseVector3);

    gtsam::Vector priorposNoiseVector12(12);
    // priorposNoiseVector12 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000,
    //                         prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000;
    priorposNoiseVector12 << prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise,
                            prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorposNoise = gtsam::noiseModel::Diagonal::Variances(priorposNoiseVector12);

    // gtsam::Vector priorvelNoiseVector3(3);
    // priorvelNoiseVector3 << prior_noise, prior_noise, prior_noise;
    // priorvelNoise = gtsam::noiseModel::Diagonal::Variances(priorvelNoiseVector3);

    gtsam::Vector priorNoiseVector6(6);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector6 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise; // prior_noise / 100, prior_noise / 100, prior_noise / 100, prior_noise / 100, prior_noise / 100, prior_noise / 100; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector priorNoiseVector12(12);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector12 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorBiasNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector12);

    gtsam::Vector priorNoiseVector3(3);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector3 << marg_noise, marg_noise, marg_noise; // 0.01, 0.01, 0.01; //, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorGravNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector3);
    
    gtsam::Vector priordtNoiseVector4(4);
    priordtNoiseVector4 << prior_noise, prior_noise, prior_noise, prior_noise;
    priordtNoise = gtsam::noiseModel::Diagonal::Variances(priordtNoiseVector4);

    // gtsam::Vector margExtNoiseVector4(4);
    // margExtNoiseVector4 << 1e-6, 1e-6, 1e-6, 1e-6;
    // margExtNoise = gtsam::noiseModel::Diagonal::Variances(margExtNoiseVector4);

    gtsam::Vector priorddtNoiseVector1(1);
    priorddtNoiseVector1 << prior_noise;  // / 10;
    priorddtNoise = gtsam::noiseModel::Diagonal::Variances(priorddtNoiseVector1);

    gtsam::Vector margrotNoiseVector3(3);
    margrotNoiseVector3 << prior_noise / 10, prior_noise / 10, prior_noise / 10;
    margrotNoise = gtsam::noiseModel::Diagonal::Variances(margrotNoiseVector3);

    gtsam::Vector margposNoiseVector6(6);
    margposNoiseVector6 << prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise; //, marg_noise, marg_noise, marg_noise; //, marg_noise, marg_noise, marg_noise;
    margposNoise = gtsam::noiseModel::Diagonal::Variances(margposNoiseVector6);

    gtsam::Vector odomaNoiseVector12(12);
    odomaNoiseVector12 <<grav_noise, grav_noise, grav_noise, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10;
    // odomaNoiseVector12 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise;
    odomaNoise = gtsam::noiseModel::Diagonal::Variances(odomaNoiseVector12);

    gtsam::Vector priorextrotNoiseVector3(3);
    priorextrotNoiseVector3 << prior_noise, prior_noise, prior_noise; // 10, 10, 100; // 
    priorextrotNoise = gtsam::noiseModel::Diagonal::Variances(priorextrotNoiseVector3);

    gtsam::Vector margNoiseVector3(3);
    margNoiseVector3 << prior_noise / 10, prior_noise / 10, prior_noise / 10; //, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, 
                        // marg_noise, marg_noise;
    margNoise = gtsam::noiseModel::Diagonal::Variances(margNoiseVector3);

    gtsam::Vector margdtNoiseVector4(4);
    margdtNoiseVector4 << prior_noise, prior_noise, prior_noise, prior_noise;
    margdtNoise = gtsam::noiseModel::Diagonal::Variances(margdtNoiseVector4);

    gtsam::Vector priorextposNoiseVector3(3);
    priorextposNoiseVector3 << prior_noise, prior_noise, prior_noise;
    priorextposNoise = gtsam::noiseModel::Diagonal::Variances(priorextposNoiseVector3);

    gtsam::Vector margddtNoiseVector1(1);
    margddtNoiseVector1 << prior_noise; //, prior_noise, prior_noise; //prior_noise;
    margddtNoise = gtsam::noiseModel::Diagonal::Variances(margddtNoiseVector1);

    gtsam::Vector dtNoiseVector4(4);
    dtNoiseVector4 << dt_noise / 10, dt_noise / 10, dt_noise / 10, dt_noise / 10;
    dtNoise = gtsam::noiseModel::Diagonal::Variances(dtNoiseVector4);

    // gtsam::Vector dtNoiseVector4_init(4);
    // dtNoiseVector4_init << dt_noise * 0.1, dt_noise * 0.1, dt_noise * 0.1, dt_noise * 0.1;
    // dtNoise_init = gtsam::noiseModel::Diagonal::Variances(dtNoiseVector4_init);

    gtsam::Vector ddtNoiseVector1(1);
    ddtNoiseVector1 << ddt_noise;
    ddtNoise = gtsam::noiseModel::Diagonal::Variances(ddtNoiseVector1);

    // gtsam::Vector ddtNoiseVector1_init(1);
    // ddtNoiseVector1_init << ddt_noise * 0.1;
    // ddtNoise_init = gtsam::noiseModel::Diagonal::Variances(ddtNoiseVector1_init);

    gtsam::Vector odomNoiseVector27(27);
    odomNoiseVector27 << grav_noise, grav_noise, grav_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, 
                            marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise;
    odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector27); // should be related to the time, maybe proportional
    // gtsam::Vector relatNoiseVector6(6);
    // relatNoiseVector6 << odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise; //, odo_noise, odo_noise, odo_noise;
    // relatNoise = gtsam::noiseModel::Diagonal::Variances(relatNoiseVector6);
    // gtsam::Vector odomNoiseVector3(3);
    // odomNoiseVector3 << odo_noise, odo_noise, odo_noise; //, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise;
    // odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector3); // should be related to the imu noise
    gtsam::Vector odomNoiseVector15(15);
    odomNoiseVector15 << odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise,
                        odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise;
    odomNoiseIMU = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector15); // should be related to the imu noise
    // odomNoiseIMU = gtsam::noiseModel::Robust::Create(
    //                 gtsam::noiseModel::mEstimator::Cauchy::Create(10), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                 gtsam::noiseModel::Diagonal::Variances(odomNoiseVector15));

    // double psrNoiseScore = psr_dopp_noise; // constant is ok...
    // double doppNoiseScore = psr_dopp_noise; // constant is ok...
    gtsam::Vector robustpsrdoppNoiseVector2(2); // gtsam::Pose3 factor has 6 elements (6D)
    // gtsam::Vector robustpsrdoppNoiseVector2_init(2); // gtsam::Pose3 factor has 6 elements (6D)
    robustpsrdoppNoiseVector2 << psr_dopp_noise, psr_dopp_noise; // / 10;
    // robustpsrdoppNoiseVector2_init << psr_dopp_noise * 10, psr_dopp_noise * 10;
    // double cpNoiseScore = cp_noise; // 1e9
    gtsam::Vector robustcpNoiseVector1(1); // gps factor has 3 elements (xyz)
    // gtsam::Vector robustcpNoiseVector1_init(1); // gps factor has 3 elements (xyz)
    robustcpNoiseVector1 << cp_noise; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    // robustcpNoiseVector1_init << cp_noise * 10; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    if (outlier_rej)
    {
      robustpsrdoppNoise = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2));


      robustpsrdoppNoise_init = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2));

      robustcpNoise = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustcpNoiseVector1));

      robustcpNoise_init = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustcpNoiseVector1));
    //   ddtNoise = gtsam::noiseModel::Robust::Create(
    //                   gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   gtsam::noiseModel::Diagonal::Variances(ddtNoiseVector1));
    //   dtNoise = gtsam::noiseModel::Robust::Create(
    //                   gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   gtsam::noiseModel::Diagonal::Variances(dtNoiseVector4));
    }
    else
    {
      robustpsrdoppNoise = gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2);
      robustcpNoise = gtsam::noiseModel::Diagonal::Variances(robustcpNoiseVector1);
    }
    // testNoise = gtsam::noiseModel::Gaussian::Covariance()
    // robustpsrdoppNoise = gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2);

} // initNoises

void GNSSAssignment::Ephemfromrinex(const std::string &rinex_filepath)
{
  rinex2ephems(rinex_filepath, sat2ephem_rnx);
  std::map<uint32_t, std::vector<EphemBasePtr>>::iterator it;
  for (it = sat2ephem_rnx.begin(); it != sat2ephem_rnx.end(); it++)
  {
    for (int j = 0; j < it->second.size(); j++)
    {
      sat2time_index_rnx[it->first].emplace(time2sec(it->second[j]->toe), j);
    }
  }
  rinex2iono_params(rinex_filepath, latest_gnss_iono_params);
}

void GNSSAssignment::Obsfromrinex(const std::string &rinex_filepath, std::queue<std::vector<ObsPtr>> &rinex_meas)
{
  rinex2obs(rinex_filepath, rinex_meas);
}

void GNSSAssignment::inputEphem(EphemBasePtr ephem_ptr) // 
{
    double toe = time2sec(ephem_ptr->toe);
    // if a new ephemeris comes
    if (sat2time_index.count(ephem_ptr->sat) == 0 || sat2time_index.at(ephem_ptr->sat).count(toe) == 0)
    {
        sat2ephem[ephem_ptr->sat].emplace_back(ephem_ptr);
        sat2time_index[ephem_ptr->sat].emplace(toe, sat2ephem.at(ephem_ptr->sat).size()-1);
    }
}

void GNSSAssignment::rinex2iono_params(const std::string &rinex_filepath, std::vector<double> &iono_params)
{
    iono_params.resize(8);
    std::ifstream file(rinex_filepath);
    std::string line;

    // check first line, mainly RINEX version
    if (!(std::getline(file, line) && line.find("RINEX VERSION")  != std::string::npos 
            && line.find("3.04") != std::string::npos))
    {
        LOG(ERROR) << "Only RINEX 3.04 is supported";
        return;
    }

    bool find_alpha = false, find_beta = false;
    while(std::getline(file, line))
    {
        if (line.find("IONOSPHERIC CORR") != std::string::npos && line.find("GPSA") != std::string::npos)
        {

            // parse ion alpha value
            for (size_t i = 0; i < 4; ++i)
            {
                std::string value_str = line.substr(i*12+5, 12);
                iono_params[i] = str2double(value_str);
            }
            find_alpha = true;
        }
        else if (line.find("IONOSPHERIC CORR") != std::string::npos && line.find("GPSB") != std::string::npos)
        {
            // parse ion beta value
            for (size_t i = 0; i < 4; ++i)
            {
                std::string value_str = line.substr(i*12+5, 12);
                iono_params[i+4] = str2double(value_str);
            }
            find_beta = true;
        }

        if(find_alpha && find_beta)
            break;
    }
    file.close();
}

void GNSSAssignment::rinex2ephems(const std::string &rinex_filepath, std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem_)
{
    uint32_t gpst_leap_seconds = static_cast<uint32_t>(-1);
    std::ifstream ephem_file(rinex_filepath);
    std::string line;
    while(std::getline(ephem_file, line))
    {
        if (line.find("RINEX VERSION / TYPE") != std::string::npos && line.find("3.04") == std::string::npos)
        {
            LOG(ERROR) << "Only RINEX 3.04 is supported for observation file";
            return;
        }
        else if (line.find("LEAP SECONDS") != std::string::npos && line.find("BDS") == std::string::npos)
            gpst_leap_seconds = static_cast<uint32_t>(std::stoi(line.substr(4, 6)));
        else if (line.find("END OF HEADER") != std::string::npos)
            break;
    }
    LOG_IF(FATAL, gpst_leap_seconds == static_cast<uint32_t>(-1)) << "No leap second record found";

    while(std::getline(ephem_file, line))
    {
        if (line.at(0) == 'G' || line.at(0) == 'C' || line.at(0) == 'E')
        {
            std::vector<std::string> ephem_lines;
            ephem_lines.push_back(line);
            for (size_t i = 0; i < 7; ++i)
            {
                std::getline(ephem_file, line);
                ephem_lines.push_back(line);
            }
            EphemPtr ephem = rinex_line2ephem(ephem_lines);
            if (!ephem || ephem->ttr.time == 0)  continue;
            if (sat2ephem_.count(ephem->sat) == 0)
                sat2ephem_.emplace(ephem->sat, std::vector<EphemBasePtr>());
            sat2ephem_.at(ephem->sat).push_back(ephem);
        }
        else if (line.at(0) == 'R')
        {
            std::vector<std::string> ephem_lines;
            ephem_lines.push_back(line);
            for (size_t i = 0; i < 3; ++i)
            {
                std::getline(ephem_file, line);
                ephem_lines.push_back(line);
            }
            GloEphemPtr glo_ephem = rinex_line2glo_ephem(ephem_lines, gpst_leap_seconds);
            if (sat2ephem_.count(glo_ephem->sat) == 0)
                sat2ephem_.emplace(glo_ephem->sat, std::vector<EphemBasePtr>());
            sat2ephem_.at(glo_ephem->sat).push_back(glo_ephem);
        }
    }
}

void GNSSAssignment::processGNSSBase(const std::vector<ObsPtr> &gnss_meas, std::vector<double> &psr_meas, std::vector<ObsPtr> &valid_meas, std::vector<EphemBasePtr> &valid_ephems, bool gnss_ready, Eigen::Vector3d ecef_pos, double last_gnss_time_process)
{
  std::vector<ObsPtr> backup_meas;
  std::vector<EphemBasePtr> backup_ephems;
  std::vector<double> backup_psr_meas;
  backup_meas.clear();
  backup_ephems.clear();
  backup_psr_meas.clear();
  const int n = 20;
  bool diff_angle[n];
  std::fill(diff_angle, diff_angle + n, false);
  for (auto obs : gnss_meas)
  {
    // filter according to system
    uint32_t sys = satsys(obs->sat, NULL);
    if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS)
        continue;
    size_t ephem_index = -1;
    EphemBasePtr best_ephem_cur;
    double obs_time = time2sec(obs->time);
    if (obs->freqs.empty())    continue;       // no valid signal measurement
    freq_idx_ = -1;
    double freq = L1_freq(obs, &freq_idx_); // L1_freq NEEDED
    if (freq_idx_ < 0)   continue;              // no L1 observation
    // printf("cn0:%f\n", obs->CN0[freq_idx_]);
    // fout_std << setw(20) << obs_time << " " << sys << " " << obs->sat << " " << obs->psr_std[freq_idx_] << " " << obs->dopp_std[freq_idx_] * LIGHT_SPEED / freq << " " << obs->cp_std[freq_idx_] * LIGHT_SPEED / freq << endl;
    // printf("%f,%f,%f\n", obs->psr[freq_idx_], obs->dopp[freq_idx_] * LIGHT_SPEED / freq, LIGHT_SPEED / freq);
    // num_std++;
    // ave_std = double(num_std - 1) / double(num_std) * ave_std + 1 / double(num_std) * obs->psr_std[freq_idx_];
    double dis_integer = obs->cp[freq_idx_] * LIGHT_SPEED / freq - obs->psr[freq_idx_];
    if (gnss_ready)
    {
        if (obs->psr_std[freq_idx_]  > gnss_psr_std_threshold ||
            obs->dopp_std[freq_idx_] > gnss_dopp_std_threshold ||
            obs->cp_std[freq_idx_] > gnss_cp_std_threshold )
        {
            sat_track_status[obs->sat] = 0;
            continue;
        }
        else
        {
            if (sat_track_status.count(obs->sat) == 0)
            {
                sat_track_status[obs->sat] = 0;
                sat_track_time[obs->sat] = obs_time;
                sat_track_last_time[obs->sat] = obs_time;
                sum_d = dis_integer;
                sum_d2 = sum_d * sum_d;
                hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
            else
            {
                if (sat_track_status[obs->sat] == 0)
                {
                    sat_track_status[obs->sat] = 0;
                    sat_track_time[obs->sat] = obs_time;
                    sat_track_last_time[obs->sat] = obs_time;
                    sum_d = dis_integer;
                    sum_d2 = sum_d * sum_d;
                    hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                    // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                    last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
                }
            }
            ++ sat_track_status[obs->sat];
        }
    }
    else
    {
        if (obs->psr_std[freq_idx_]  > gnss_psr_std_threshold||
            obs->dopp_std[freq_idx_] > gnss_dopp_std_threshold ||
            obs->cp_std[freq_idx_] > gnss_cp_std_threshold)
        {
            sat_track_status[obs->sat] = 0;
            continue;
        }
        else
        {
            if (sat_track_status.count(obs->sat) == 0)
            {
                sat_track_status[obs->sat] = 0;
                sat_track_last_time[obs->sat] = obs_time;
                sat_track_time[obs->sat] = obs_time;
                sum_d = dis_integer;
                sum_d2 = sum_d * sum_d;
                hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
            else
            {
                if (sat_track_status[obs->sat] == 0)
                {
                    sat_track_status[obs->sat] = 0;
                    sat_track_time[obs->sat] = obs_time;
                    sat_track_last_time[obs->sat] = obs_time;
                    sum_d = dis_integer;
                    sum_d2 = sum_d * sum_d;
                    hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                    // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                    last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
                }
            }
            ++ sat_track_status[obs->sat];
            // sat_track_last_time[obs->sat] = obs_time;
        }
    }
    
    if (last_cp_meas[obs->sat] < 100)
    {
        sat_track_status[obs->sat] = 0;
        hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
        // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
    }
    else
    {
    if (obs_time - sat_track_last_time[obs->sat] > 15)
    {
        sat_track_status[obs->sat] = 1;
        sat_track_last_time[obs->sat] = obs_time;
        sat_track_time[obs->sat] = obs_time;
        sum_d = dis_integer; 
        sum_d2 = sum_d * sum_d;
        hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
        // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
        last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
    }
    else
    {
        if (sat_track_status[obs->sat] > 1) // problem!
        {
            // if (obs->status[freq_idx_])
            if (fabs(dis_integer) > 6 * sqrt(sum_d2 / sat_track_status[obs->sat] - sum_d * sum_d / sat_track_status[obs->sat] / sat_track_status[obs->sat])) // ?
            {
                sat_track_status[obs->sat] = 1;
                sat_track_last_time[obs->sat] = obs_time;
                sat_track_time[obs->sat] = obs_time;
                sum_d = dis_integer; 
                sum_d2 = sum_d * sum_d;
                hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
            else
            {
                sum_d += dis_integer;
                sum_d2 += dis_integer * dis_integer;
                sat_track_last_time[obs->sat] = obs_time;
                // if (last_cp_meas[obs->sat] > 10 && obs->cp[freq_idx_] * LIGHT_SPEED / freq > 10)
                {
                    double last_psr = hatch_filter_meas[obs->sat];
                    hatch_filter_meas[obs->sat] = 1 / double(sat_track_status[obs->sat]) * obs->psr[freq_idx_] + double(sat_track_status[obs->sat]-1)/double(sat_track_status[obs->sat]) 
                                        * (last_psr + obs->cp[freq_idx_] * LIGHT_SPEED / freq - last_cp_meas[obs->sat]); // obs->psr[freq_idx_];
                    obs->psr_std[freq_idx_] = std::sqrt(obs->psr_std[freq_idx_] * obs->psr_std[freq_idx_] / 2 + obs->cp_std[freq_idx_] * obs->cp_std[freq_idx_] * LIGHT_SPEED / freq * LIGHT_SPEED / freq);
                    // cout << "check after:" << hatch_filter_meas[obs->sat] << ";" << obs->psr[freq_idx_] << ";" << obs->cp[freq_idx_] * LIGHT_SPEED / freq << ";" << last_cp_meas[obs->sat] << endl;
                }
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
        }
    }
    }
    if (!ephem_from_rinex)
    {
      // if not got cooresponding ephemeris yet
      if (sat2ephem.count(obs->sat) == 0)
          continue;
      
      std::map<double, size_t> time2index = sat2time_index.at(obs->sat);
      double ephem_time = EPH_VALID_SECONDS;
      for (auto ti : time2index)
      {
          if (std::abs(ti.first - obs_time) < ephem_time)
          {
              ephem_time = std::abs(ti.first - obs_time);
              ephem_index = ti.second;
          }
      }
      std::map<double, size_t>().swap(time2index);
      if (ephem_time >= EPH_VALID_SECONDS)
      {
          cerr << "ephemeris not valid anymore\n";
          continue;
      }
      best_ephem_cur = sat2ephem.at(obs->sat).at(ephem_index);
    }
    else
    {
      // cout << "gnss ready:" << gnss_ready << endl;
      if (sat2ephem_rnx.count(obs->sat) == 0)
          continue;
    //   if (obs->freqs.empty())    continue;       // no valid signal measurement
      
    //   freq_idx_ = -1;
    //   L1_freq(obs, &freq_idx_);
    //   if (freq_idx_ < 0)   continue;              // no L1 observation
      
    //   obs_time = time2sec(obs->time);
      std::map<double, size_t> time2index = sat2time_index_rnx.at(obs->sat);
      double ephem_time = EPH_VALID_SECONDS;
      for (auto ti : time2index)
      {
        //   cout << std::abs(ti.first - obs_time) << endl;
          if (std::abs(ti.first - obs_time) < ephem_time)
          {
              ephem_time = std::abs(ti.first - obs_time);
              ephem_index = ti.second;
          }
      }
      std::map<double, size_t>().swap(time2index);
      if (ephem_time >= EPH_VALID_SECONDS)
      {
          cerr << "ephemeris not valid anymore\n";
          continue;
      }
      best_ephem_cur = sat2ephem_rnx.at(obs->sat).at(ephem_index);
    }
      const EphemBasePtr &best_ephem = best_ephem_cur;
      // filter by tracking status
      LOG_IF(FATAL, freq_idx_ < 0) << "No L1 observation found.\n";
      
      // filter by elevation angle
      if (gnss_ready) // && !quick_it) // gnss initialization is completed, then filter the sat by elevation angle // need to be defined
      {
          Eigen::Vector3d sat_ecef;
          if (sys == SYS_GLO)
              sat_ecef = geph2pos(obs->time, std::dynamic_pointer_cast<GloEphem>(best_ephem), NULL);
          else
              sat_ecef = eph2pos(obs->time, std::dynamic_pointer_cast<Ephem>(best_ephem), NULL);
          double azel[2] = {0, M_PI/2.0};
        //   if (fabs((ecef_pos-sat_ecef).norm() - hatch_filter_meas[obs->sat]) > 3 * 1e6)
            //   continue;
          sat_azel(ecef_pos, sat_ecef, azel); // ecef_pos should be updated for this time step // coarse value is acceptable as well TODO
        //   std::cout << "check angle:" << azel[0] << ";" << azel[1] << std::endl;
          if (azel[1] < gnss_elevation_threshold*M_PI/180.0)
              continue;
          int angle_id = int(azel[0] / 0.314);
          if (diff_angle[angle_id])
          {
            backup_meas.push_back(obs);
            backup_ephems.push_back(best_ephem);
            backup_psr_meas.push_back(hatch_filter_meas[obs->sat]);
            continue;
          }
          diff_angle[angle_id] = true;
      }
      psr_meas.push_back(hatch_filter_meas[obs->sat]); // obs->psr[freq_idx_]); // 
    //   obs->psr[freq_idx_] = hatch_filter_meas[obs->sat];
      valid_meas.push_back(obs);
      valid_ephems.push_back(best_ephem);
    if (valid_meas.size() >= 15) break; // 
  }
  if (valid_meas.size() < 15) 
  {
    // for (int i = 0; i < backup_meas.size(); i++)
    // {
    //   valid_meas.push_back(backup_meas[i]);
    //   valid_ephems.push_back(backup_ephems[i]);
    //   psr_meas.push_back(backup_psr_meas[i]);
    //   if (valid_meas.size() >= 15) break;
    // }
  } //
}

void GNSSAssignment::delete_variables(bool nolidar, size_t frame_delete, int frame_num, size_t &id_accumulate, gtsam::FactorIndices delete_factor)
{
    if (!nolidar)
    {
      if (frame_delete > 0)
      {
        if (frame_delete >= change_ext)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedERNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(P(0))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedEPNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(E(0))); // important
            gtsam::PriorFactor<gtsam::Rot3> init_ER(P(0),isamCurrentEstimate.at<gtsam::Rot3>(P(0)), updatedERNoise); // margrotNoise); // 
            gtsam::PriorFactor<gtsam::Vector3> init_EP(E(0),isamCurrentEstimate.at<gtsam::Vector3>(E(0)), updatedEPNoise); // margNoise); // 
            gtSAMgraph.add(init_ER);
            gtSAMgraph.add(init_EP);
            // factor_id_frame[0].push_back(id_accumulate);
            // factor_id_frame[0].push_back(id_accumulate+1);
            factor_id_frame[frame_num - 1 - frame_delete].push_back(id_accumulate);
            factor_id_frame[frame_num - 1 - frame_delete].push_back(id_accumulate+1);
            id_accumulate += 2;
            change_ext = frame_num;
        }
        size_t j = 0;
        for (; j < marg_thred; j++)
        {
            // get updated noise before reset
            // gtsam::noiseModel::Gaussian::shared_ptr updatedRotNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(R(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(A(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(F(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedDtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(B(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedDdtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(C(frame_delete+j))); // important

            // gtsam::PriorFactor<gtsam::Rot3> init_rot(R(frame_delete+j),isamCurrentEstimate.at<gtsam::Rot3>(R(frame_delete+j)),updatedRotNoise); //  margrotNoise); // 
            // gtsam::PriorFactor<gtsam::Vector12> init_vel(F(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector12>(F(frame_delete+j)), updatedPosNoise); // margposNoise);
            // gtsam::PriorFactor<gtsam::Vector6> init_vel(A(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector6>(A(frame_delete+j)),updatedPosNoise); //  margposNoise); // 
            gtsam::PriorFactor<gtsam::Vector4> init_dt(B(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector4>(B(frame_delete+j)), margdtNoise); //  updatedDtNoise); // 
            gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector1>(C(frame_delete+j)), margddtNoise); // updatedDdtNoise); // 
            // gtSAMgraph.add(init_rot);
            // gtSAMgraph.add(init_vel);
            gtSAMgraph.add(init_dt);
            gtSAMgraph.add(init_ddt);
            factor_id_frame[0].push_back(id_accumulate+(j)*2);
            factor_id_frame[0].push_back(id_accumulate+1+(j)*2);
            // factor_id_frame[0].push_back(id_accumulate+2+(j)*4);
            // factor_id_frame[0].push_back(id_accumulate+3+(j)*4);
        }
        // id_accumulate += (j-1) * 4;
        id_accumulate += j * 2;
      }
      isam.update(gtSAMgraph, initialEstimate);
      gtSAMgraph.resize(0); // will the initialEstimate change?
      initialEstimate.clear();
      isam.update(gtSAMgraph, initialEstimate, delete_factor);   
    }
    else
    {
      if (frame_delete > 0) // (frame_delete == 0)
      {
        size_t j = 0;
        // for (; j < 10; j++)
        for (; j < marg_thred; j++)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedRotNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(R(frame_delete+j))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(F(frame_delete+j))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedDtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(B(frame_delete+j))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedDdtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(C(frame_delete+j))); // important

            gtsam::PriorFactor<gtsam::Rot3> init_rot(R(frame_delete+j),isamCurrentEstimate.at<gtsam::Rot3>(R(frame_delete+j)), updatedRotNoise); // margrotNoise);
            gtsam::PriorFactor<gtsam::Vector12> init_vel(F(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector12>(F(frame_delete+j)), updatedPosNoise); // margposNoise);
            gtsam::PriorFactor<gtsam::Vector4> init_dt(B(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector4>(B(frame_delete+j)), updatedDtNoise); // margdtNoise);
            gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector1>(C(frame_delete+j)), updatedDdtNoise); // margddtNoise); // could delete?
            gtSAMgraph.add(init_rot);
            gtSAMgraph.add(init_vel);
            gtSAMgraph.add(init_dt);
            gtSAMgraph.add(init_ddt);
            
            {
            factor_id_frame[0].push_back(id_accumulate+j*4);
            factor_id_frame[0].push_back(id_accumulate+1+j*4);
            factor_id_frame[0].push_back(id_accumulate+2+j*4);
            factor_id_frame[0].push_back(id_accumulate+3+j*4);
            // factor_id_frame[0].push_back(id_accumulate+4+j*4);
            }
        }
        id_accumulate += j * 4;
      }
      isam.update(gtSAMgraph, initialEstimate);
      gtSAMgraph.resize(0); // will the initialEstimate change?
      initialEstimate.clear();
      isam.update(gtSAMgraph, initialEstimate, delete_factor);
      // gtSAMgraph.resize(0); // will the initialEstimate change?
      // initialEstimate.clear();
      // isam.update();
    }
}  

double GNSSAssignment::str2double(const std::string &num_str)
{
    size_t D_pos = num_str.find("D");
    std::string tmp_str = num_str;
    if (D_pos != std::string::npos)
        tmp_str = tmp_str.replace(D_pos, 1, "e");
    return std::stod(tmp_str);
}

EphemPtr GNSSAssignment::rinex_line2ephem(const std::vector<std::string> &ephem_lines)
{
    LOG_IF(FATAL, ephem_lines.size() != 8) << "Ephemeris record should contain 8 lines";
    uint32_t sat_sys = SYS_NONE;
    if      (ephem_lines[0].at(0) == 'G')    sat_sys = SYS_GPS;
    else if (ephem_lines[0].at(0) == 'C')    sat_sys = SYS_BDS;
    else if (ephem_lines[0].at(0) == 'E')    sat_sys = SYS_GAL;
    LOG_IF(FATAL, sat_sys == SYS_NONE) << "Satellite system is not supported: " << ephem_lines[0].at(0);

    EphemPtr ephem(new Ephem());
    uint32_t prn = static_cast<uint32_t>(std::stoi(ephem_lines[0].substr(1, 2)));
    {
        ephem->sat = sat_no(sat_sys, prn);
    }
    {
        ephem->sat = sat_no(sat_sys, prn);
    }
    double epoch[6];
    epoch[0] = static_cast<double>(std::stoi(ephem_lines[0].substr(4, 4)));
    epoch[1] = static_cast<double>(std::stoi(ephem_lines[0].substr(9, 2)));
    epoch[2] = static_cast<double>(std::stoi(ephem_lines[0].substr(12, 2)));
    epoch[3] = static_cast<double>(std::stoi(ephem_lines[0].substr(15, 2)));
    epoch[4] = static_cast<double>(std::stoi(ephem_lines[0].substr(18, 2)));
    epoch[5] = static_cast<double>(std::stoi(ephem_lines[0].substr(21, 2)));
    ephem->toc = epoch2time(epoch);
    if (sat_sys == SYS_BDS)     ephem->toc.time += 14;     // BDS-GPS time correction
    ephem->af0 = str2double(ephem_lines[0].substr(23, 19));
    ephem->af1 = str2double(ephem_lines[0].substr(42, 19));
    ephem->af2 = str2double(ephem_lines[0].substr(61, 19));

    // the second line
    if (sat_sys == SYS_GPS)
        ephem->iode  = str2double(ephem_lines[1].substr(4, 19));
    ephem->crs       = str2double(ephem_lines[1].substr(23, 19));
    ephem->delta_n   = str2double(ephem_lines[1].substr(42, 19));
    ephem->M0        = str2double(ephem_lines[1].substr(61, 19));

    // the third line
    ephem->cuc = str2double(ephem_lines[2].substr(4, 19));
    ephem->e = str2double(ephem_lines[2].substr(23, 19));
    ephem->cus = str2double(ephem_lines[2].substr(42, 19));
    double sqrt_A = str2double(ephem_lines[2].substr(61, 19));
    ephem->A = sqrt_A * sqrt_A;

    // the forth line
    ephem->toe_tow = str2double(ephem_lines[3].substr(4, 19));
    ephem->cic = str2double(ephem_lines[3].substr(23, 19));
    ephem->OMG0 = str2double(ephem_lines[3].substr(42, 19));
    ephem->cis = str2double(ephem_lines[3].substr(61, 19));

    // the fifth line
    ephem->i0 = str2double(ephem_lines[4].substr(4, 19));
    ephem->crc = str2double(ephem_lines[4].substr(23, 19));
    ephem->omg = str2double(ephem_lines[4].substr(42, 19));
    ephem->OMG_dot = str2double(ephem_lines[4].substr(61, 19));

    // the sixth line
    ephem->i_dot = str2double(ephem_lines[5].substr(4, 19));
    if  (sat_sys == SYS_GAL)
    {
        uint32_t ephe_source = static_cast<uint32_t>(str2double(ephem_lines[5].substr(23, 19)));
        if (!(ephe_source & 0x01))  
        {
            // LOG(ERROR) << "not contain I/NAV E1-b info, skip this ephemeris";
            return ephem;   // only parse I/NAV E1-b ephemeris
        }
    }
    ephem->week = static_cast<uint32_t>(str2double(ephem_lines[5].substr(42, 19)));
    if (sat_sys == SYS_GPS || sat_sys == SYS_GAL)     ephem->toe = gpst2time(ephem->week, ephem->toe_tow);
    else if (sat_sys == SYS_BDS)                      ephem->toe = bdt2time(ephem->week, ephem->toe_tow+14);
    // if (sat_sys == SYS_GAL)     ephem->toe = gst2time(ephem->week, ephem->toe_tow);

    // the seventh line
    ephem->ura = str2double(ephem_lines[6].substr(4, 19));
    ephem->health = static_cast<uint32_t>(str2double(ephem_lines[6].substr(23, 19)));
    ephem->tgd[0] = str2double(ephem_lines[6].substr(42, 19));
    if (sat_sys == SYS_BDS || sat_sys == SYS_GAL)
        ephem->tgd[1] = str2double(ephem_lines[6].substr(61, 19));
    if (sat_sys == SYS_GPS)     ephem->iodc = str2double(ephem_lines[6].substr(61, 19));

    // the eighth line
    double ttr_tow = str2double(ephem_lines[7].substr(4, 19));
    // GAL week = GST week + 1024 + rollover, already align with GPS week!!!
    if      (sat_sys == SYS_GPS || sat_sys == SYS_GAL)   ephem->ttr = gpst2time(ephem->week, ttr_tow);
    else if (sat_sys == SYS_BDS)   ephem->ttr = bdt2time(ephem->week, ttr_tow);

    // convert time system to parameter GPST
    if (sat_sys == SYS_BDS)
    {
        uint32_t week = 0;
        ephem->toe_tow = time2gpst(ephem->toe, &week);
        ephem->week = week;
    }

    return ephem;
}

GloEphemPtr GNSSAssignment::rinex_line2glo_ephem(const std::vector<std::string> &ephem_lines, const uint32_t gpst_leap_seconds)
{
    LOG_IF(FATAL, ephem_lines.size() != 4) << "GLO ephemeris record should contain 8 lines";
    LOG_IF(FATAL, ephem_lines[0].at(0) != 'R') << "Not a valid GLO ephemeris record";
    GloEphemPtr glo_ephem(new GloEphem());

    uint32_t prn = static_cast<uint32_t>(std::stoi(ephem_lines[0].substr(1, 2)));
    {
        glo_ephem->sat = sat_no(SYS_GLO, prn);
    }
    {
        glo_ephem->sat = sat_no(SYS_GLO, prn);
    }
    double epoch[6];
    epoch[0] = static_cast<double>(std::stoi(ephem_lines[0].substr(4, 4)));
    epoch[1] = static_cast<double>(std::stoi(ephem_lines[0].substr(9, 2)));
    epoch[2] = static_cast<double>(std::stoi(ephem_lines[0].substr(12, 2)));
    epoch[3] = static_cast<double>(std::stoi(ephem_lines[0].substr(15, 2)));
    epoch[4] = static_cast<double>(std::stoi(ephem_lines[0].substr(18, 2)));
    epoch[5] = static_cast<double>(std::stoi(ephem_lines[0].substr(21, 2)));
    glo_ephem->toe = epoch2time(epoch);
    glo_ephem->toe.time += gpst_leap_seconds;
    glo_ephem->tau_n = -1.0 * str2double(ephem_lines[0].substr(23, 19));
    glo_ephem->gamma = str2double(ephem_lines[0].substr(42, 19));

    // the second line
    glo_ephem->pos[0] = str2double(ephem_lines[1].substr(4, 19)) * 1e3;
    glo_ephem->vel[0] = str2double(ephem_lines[1].substr(23, 19)) * 1e3;
    glo_ephem->acc[0] = str2double(ephem_lines[1].substr(42, 19)) * 1e3;
    glo_ephem->health = static_cast<uint32_t>(str2double(ephem_lines[1].substr(61, 19)));

    // the third line
    glo_ephem->pos[1] = str2double(ephem_lines[2].substr(4, 19)) * 1e3;
    glo_ephem->vel[1] = str2double(ephem_lines[2].substr(23, 19)) * 1e3;
    glo_ephem->acc[1] = str2double(ephem_lines[2].substr(42, 19)) * 1e3;
    glo_ephem->freqo  = static_cast<int>(str2double(ephem_lines[2].substr(61, 19)));

    // the forth line
    glo_ephem->pos[2] = str2double(ephem_lines[3].substr(4, 19)) * 1e3;
    glo_ephem->vel[2] = str2double(ephem_lines[3].substr(23, 19)) * 1e3;
    glo_ephem->acc[2] = str2double(ephem_lines[3].substr(42, 19)) * 1e3;
    glo_ephem->age  = static_cast<uint32_t>(str2double(ephem_lines[3].substr(61, 19)));

    return glo_ephem;
}

ObsPtr GNSSAssignment::rinex_line2obs(const std::string rinex_str, 
    const std::map<uint8_t, std::vector<std::string>> &sys2type)
{
    ObsPtr obs;
    uint8_t sys_char = rinex_str.at(0);
    if (char2sys.count(sys_char) == 0)   return obs;
    obs.reset(new Obs());
    uint32_t sys = char2sys.at(sys_char);
    uint32_t prn = static_cast<uint32_t>(std::stoi(rinex_str.substr(1, 2)));
    obs->sat = sat_no(sys, prn);
    std::map<double, uint32_t> freq2idx;
    std::vector<std::string> date_types = sys2type.at(sys_char);
    uint32_t line_offset = 3;
    for (auto type : date_types)
    {
        std::string field_str = rinex_str.substr(line_offset, 14);
        line_offset += 14 + 2;
        if (field_str.find_first_not_of(' ') == std::string::npos)  continue;
        const double field_value = stod(field_str);
        std::stringstream ss;
        ss << sys_char << type.at(1);
        const double freq = type2freq.at(ss.str());
        uint32_t freq_idx = static_cast<uint32_t>(-1);
        if (freq2idx.count(freq) == 0)
        {
            obs->freqs.push_back(freq);
            freq_idx = obs->freqs.size()-1;
            freq2idx.emplace(freq, freq_idx);
        }
        else
        {
            freq_idx = freq2idx.at(freq);
        }
        
        if (type.at(0) == 'L')
            obs->cp[freq_idx] = field_value;
        else if (type.at(0) == 'C')
            obs->psr[freq_idx] = field_value;
        else if (type.at(0) == 'D')
            obs->dopp[freq_idx] = field_value;
        else if (type.at(0) == 'S')
            obs->CN0[freq_idx] = field_value;
        else
            LOG(FATAL) << "Unrecognized measurement type " << type.at(0);
    }
    // fill in other fields
    uint32_t num_freqs = obs->freqs.size();
    LOG_IF(FATAL, num_freqs < obs->CN0.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->CN0), num_freqs-obs->CN0.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->LLI.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->LLI), num_freqs-obs->LLI.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->code.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->code), num_freqs-obs->code.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->psr.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->psr), num_freqs-obs->psr.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->psr_std.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->psr_std), num_freqs-obs->psr_std.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->cp.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->cp), num_freqs-obs->cp.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->cp_std.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->cp_std), num_freqs-obs->cp_std.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->dopp.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->dopp), num_freqs-obs->dopp.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->dopp_std.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->dopp_std), num_freqs-obs->dopp_std.size(), 0);
    LOG_IF(FATAL, num_freqs < obs->status.size()) << "Suspicious observation field.\n";
    std::fill_n(std::back_inserter(obs->status), num_freqs-obs->status.size(), 0x0F);

    return obs;
}

// TODO: GLONASS slot number
void GNSSAssignment::rinex2obs(const std::string &rinex_filepath, std::queue<std::vector<ObsPtr>> &rinex_meas)
{
    std::ifstream obs_file(rinex_filepath);
    std::string rinex_line;

    // parse header
    std::map<uint8_t, std::vector<std::string>> sys2type;
    uint8_t sys_char = 0;
    
    while (std::getline(obs_file, rinex_line))
    {
        if (rinex_line.find("RINEX VERSION / TYPE") != std::string::npos && rinex_line.find("3.04") == std::string::npos)
        {
            LOG(ERROR) << "Only RINEX 3.04 is supported for observation file";
            return;
        }
        else if (rinex_line.find("SYS / # / OBS TYPES") != std::string::npos)
        {
            if (rinex_line.at(0) != ' ')
            {
                sys_char = rinex_line.at(0);
                sys2type.emplace(sys_char, std::vector<std::string>());
            }
            for (size_t i = 0; i < 13; ++i)
                if (rinex_line.substr(7+4*i, 3) != "   ")
                    sys2type.at(sys_char).emplace_back(rinex_line.substr(7+4*i, 3));
        }
        else if (rinex_line.find("END OF HEADER") != std::string::npos)  break;
    }

    while (std::getline(obs_file, rinex_line))
    {
        LOG_IF(FATAL, rinex_line.at(0) != '>') << "Invalid Observation record " << rinex_line;
        LOG_IF(FATAL, rinex_line.at(31) != '0') << "Invalid Epoch data " << rinex_line.at(31)-48;
        std::vector<double> epoch_time;
        epoch_time.emplace_back(std::stod(rinex_line.substr(2, 4)));
        epoch_time.emplace_back(std::stod(rinex_line.substr(7, 2)));
        epoch_time.emplace_back(std::stod(rinex_line.substr(10, 2)));
        epoch_time.emplace_back(std::stod(rinex_line.substr(13, 2)));
        epoch_time.emplace_back(std::stod(rinex_line.substr(16, 2)));
        epoch_time.emplace_back(std::stod(rinex_line.substr(18, 11)));
        gtime_t obs_time = epoch2time(&(epoch_time[0]));
        const int num_obs = std::stoi(rinex_line.substr(32, 3));
        std::vector<ObsPtr> meas;
        for (int i = 0; i < num_obs; ++i)
        {
            LOG_IF(FATAL, !std::getline(obs_file, rinex_line)) << "Incomplete RINEX file";
            ObsPtr obs = rinex_line2obs(rinex_line, sys2type);
            if (!obs || obs->freqs.empty())  continue;
            obs->time = obs_time;
            meas.emplace_back(obs);
        }
        rinex_meas.push(meas); // only GPS and BDS?
    }
}

// int GNSSAssignment::satno_rtk(int sys, int prn)
// {
//     if (prn<=0) return 0;
//     switch (sys) {
//         case SYS_GPS:
//             if (prn<1||32<prn) return 0;
//             return prn-1+1;
//         case SYS_GLO:
//             if (prn<1||24<prn) return 0;
//             return 32+prn-1+1;
//         case SYS_GAL:
//             if (prn<1||30<prn) return 0;
//             return 32+24+prn-1+1;
//         case SYS_QZS:
//             if (prn<193||199<prn) return 0;
//             return 32+24+30+prn-193+1;
//         case SYS_BDS:
//             if (prn<1||35<prn) return 0;
//             return 32+24+30+7+prn-1+1;
//         case SYS_LEO:
//             if (prn<1||10<prn) return 0;
//             return 32+24+30+7+35+prn-1+1;
//         case SYS_SBS:
//             if (prn<120||140<prn) return 0;
//             return 32+24+30+7+35+10+prn-120+1;
//     }
//     return 0;
// }
