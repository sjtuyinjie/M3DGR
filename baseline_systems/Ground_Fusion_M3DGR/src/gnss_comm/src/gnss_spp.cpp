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

#include "gnss_spp.hpp"
#include "gnss_utility.hpp"
#include <glog/logging.h>

#define CUT_OFF_DEGREE 15.0

namespace gnss_comm
{
    void filter_L1(const std::vector<ObsPtr> &obs, const std::vector<EphemBasePtr> &ephems, 
        std::vector<ObsPtr> &L1_obs, std::vector<EphemBasePtr> &L1_ephems)
    {
        L1_obs.clear();
        L1_ephems.clear();
        for (size_t i = 0; i < obs.size(); ++i)
        {
            const ObsPtr &this_obs = obs[i];
            // check system
            const uint32_t sys = satsys(this_obs->sat, NULL);
            if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_BDS && sys != SYS_GAL)
                continue;
            // check signal frequency
            const double obs_freq = L1_freq(this_obs, NULL);
            if (obs_freq < 0)   continue;

            L1_obs.push_back(this_obs);
            L1_ephems.push_back(ephems[i]);
        }
    }

    std::vector<SatStatePtr> sat_states(const std::vector<ObsPtr> &obs, 
        const std::vector<EphemBasePtr> &ephems)
    {
        std::vector<SatStatePtr> all_sv_states;
        const uint32_t num_obs = obs.size();
        for (size_t i = 0; i < num_obs; ++i)
        {
            SatStatePtr sat_state(new SatState());
            all_sv_states.push_back(sat_state);

            ObsPtr this_obs = obs[i];
            const uint32_t sat = this_obs->sat;
            const uint32_t sys = satsys(sat, NULL);
            int l1_idx = -1;
            L1_freq(this_obs, &l1_idx);
            if (l1_idx < 0)   continue;
            double tof = this_obs->psr[l1_idx] / LIGHT_SPEED;

            gtime_t sv_tx = time_add(this_obs->time, -tof);
            double svdt = 0, svddt = 0;
            Eigen::Vector3d sv_pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d sv_vel = Eigen::Vector3d::Zero();
            if (sys == SYS_GLO)
            {
                GloEphemPtr glo_ephem = std::dynamic_pointer_cast<GloEphem>(ephems[i]);
                svdt = geph2svdt(sv_tx, glo_ephem);
                sv_tx = time_add(sv_tx, -svdt);
                sv_pos = geph2pos(sv_tx, glo_ephem, &svdt);
                sv_vel = geph2vel(sv_tx, glo_ephem, &svddt);
            }
            else
            {
                EphemPtr ephem = std::dynamic_pointer_cast<Ephem>(ephems[i]);
                svdt = eph2svdt(sv_tx, ephem);
                sv_tx = time_add(sv_tx, -svdt);
                sv_pos = eph2pos(sv_tx, ephem, &svdt);
                sv_vel = eph2vel(sv_tx, ephem, &svddt);
                sat_state->tgd = ephem->tgd[0];
            }
            sat_state->sat_id = sat;
            sat_state->ttx    = sv_tx;
            sat_state->pos    = sv_pos;
            sat_state->vel    = sv_vel;
            sat_state->dt     = svdt;
            sat_state->ddt    = svddt;
        }
        return all_sv_states;
    }

    void psr_res(const Eigen::Matrix<double, 7, 1> &rcv_state, const std::vector<ObsPtr> &obs, 
        const std::vector<SatStatePtr> &all_sv_states, const std::vector<double> &iono_params, 
        Eigen::VectorXd &res, Eigen::MatrixXd &J, std::vector<Eigen::Vector2d> &atmos_delay, 
        std::vector<Eigen::Vector2d> &all_sv_azel)
    {
        const uint32_t num_sv = all_sv_states.size();
        // clear output
        res = Eigen::VectorXd::Zero(num_sv);
        J = Eigen::MatrixXd::Zero(num_sv, 7);
        atmos_delay.resize(num_sv);
        all_sv_azel.resize(num_sv);

        for (uint32_t i = 0; i < num_sv; ++i)
        {
            int l1_idx = -1;
            L1_freq(obs[i], &l1_idx);
            if (l1_idx < 0)   continue;

            const SatStatePtr &sat_state = all_sv_states[i];
            uint32_t this_sys = satsys(sat_state->sat_id, NULL);
            Eigen::Vector3d sv_pos = sat_state->pos;

            double ion_delay=0, tro_delay=0;
            double azel[2] = {0, M_PI/2.0};
            if (rcv_state.topLeftCorner<3,1>().norm() > 0)
            {
                sat_azel(rcv_state.topLeftCorner<3,1>(), sv_pos, azel);
                Eigen::Vector3d rcv_lla = ecef2geo(rcv_state.head<3>());
                // use satellite signal transmit time instead
                tro_delay = calculate_trop_delay(sat_state->ttx, rcv_lla, azel);
                ion_delay = calculate_ion_delay(sat_state->ttx, iono_params, rcv_lla, azel);
            }

            Eigen::Vector3d rv2sv = sv_pos - rcv_state.topLeftCorner<3,1>();
            Eigen::Vector3d unit_rv2sv = rv2sv.normalized();
            double sagnac_term = EARTH_OMG_GPS*(sv_pos(0)*rcv_state(1,0)-
                sv_pos(1)*rcv_state(0,0))/LIGHT_SPEED;
            double psr_estimated = rv2sv.norm() + sagnac_term + rcv_state(3+sys2idx.at(this_sys)) -
             sat_state->dt*LIGHT_SPEED + tro_delay + ion_delay + sat_state->tgd*LIGHT_SPEED;

            J.block(i, 0, 1, 3) = -unit_rv2sv.transpose();
            J(i, 3+sys2idx.at(this_sys)) = 1.0;
            res(i) = psr_estimated - obs[i]->psr[l1_idx];

            atmos_delay[i] = Eigen::Vector2d(ion_delay, tro_delay);
            all_sv_azel[i] = Eigen::Vector2d(azel[0], azel[1]);
        }
    }

    Eigen::Matrix<double, 7, 1> psr_pos(const std::vector<ObsPtr> &obs, 
        const std::vector<EphemBasePtr> &ephems, const std::vector<double> &iono_params)
    {
        Eigen::Matrix<double, 7, 1> result;
        result.setZero();

        std::vector<ObsPtr> valid_obs;
        std::vector<EphemBasePtr> valid_ephems;
        filter_L1(obs, ephems, valid_obs, valid_ephems);
        if (valid_obs.size() < 4)
        {
            LOG(ERROR) << "[gnss_comm::psr_pos] GNSS observation not enough.\n";
            return result;
        }

        std::vector<SatStatePtr> all_sat_states = sat_states(valid_obs, valid_ephems);

        Eigen::Matrix<double, 7, 1> xyzt;
        xyzt.setZero();
        double dx_norm = 1.0;
        uint32_t num_iter = 0;
        while(num_iter < MAX_ITER_PVT && dx_norm > EPSILON_PVT)
        {
            Eigen::VectorXd b;
            Eigen::MatrixXd G;
            std::vector<Eigen::Vector2d> atmos_delay;
            std::vector<Eigen::Vector2d> all_sv_azel;
            psr_res(xyzt, valid_obs, all_sat_states, iono_params, b, G, atmos_delay, all_sv_azel);

            std::vector<uint32_t> good_idx;
            for (uint32_t i = 0; i < valid_obs.size(); ++i)
            {
                if (G.row(i).norm() <= 0)   continue;       // res not computed
                if (all_sv_azel[i].y() > CUT_OFF_DEGREE/180.0*M_PI)  
                    good_idx.push_back(i);
            }
            int sys_mask[4] = {0, 0, 0, 0};
            for (uint32_t i = 0; i < valid_obs.size(); ++i)
            {
                const uint32_t obs_sys = satsys(valid_obs[i]->sat, NULL);
                sys_mask[sys2idx.at(obs_sys)] = 1;
            }
            uint32_t num_extra_constraint = 4;
            for (uint32_t k = 0; k < 4; ++k)    num_extra_constraint -= sys_mask[k];
            LOG_IF(FATAL, num_extra_constraint >= 4) << "[gnss_comm::psr_pos] too many extra-clock constraints.\n";

            const uint32_t good_num = good_idx.size();
            LOG_IF(ERROR, good_num < 4) << "too few good obs: " << good_num;

            Eigen::MatrixXd good_G(good_num+num_extra_constraint, 7);
            Eigen::VectorXd good_b(good_num+num_extra_constraint);
            Eigen::MatrixXd good_W(good_num+num_extra_constraint, good_num+num_extra_constraint);
            good_W.setZero();
            for (uint32_t i = 0; i < good_num; ++i)
            {
                const uint32_t origin_idx = good_idx[i];
                const ObsPtr &this_obs = valid_obs[origin_idx];
                good_G.row(i) = G.row(origin_idx);
                good_b(i) = b(origin_idx);
                // compose weight
                const double sin_el = sin(all_sv_azel[origin_idx].y());
                double weight = sin_el*sin_el;
                int l1_idx = -1;
                L1_freq(this_obs, &l1_idx);
                LOG_IF(FATAL, l1_idx < 0) << "[gnss_comm::psr_pos] no L1 observation found.\n";

                if (this_obs->psr_std[l1_idx] > 0)
                    weight /= (this_obs->psr_std[l1_idx]/0.16);
                const uint32_t obs_sys = satsys(this_obs->sat, NULL);
                if (obs_sys == SYS_GPS || obs_sys == SYS_BDS)
                    weight /= valid_ephems[origin_idx]->ura-1;
                else if (obs_sys == SYS_GAL)
                    weight /= valid_ephems[origin_idx]->ura-2;
                else if (obs_sys == SYS_GLO)
                    weight /= 4;
                good_W(i, i) = weight;
            }
            uint32_t tmp_count = good_num;
            // add extra pseudo measurement to contraint unobservable clock bias
            for (size_t k = 0; k < 4; ++k)
            {
                if (!sys_mask[k])
                {
                    good_G.row(tmp_count).setZero();
                    good_G(tmp_count, k+3) = 1.0;
                    good_b(tmp_count) = 0;
                    good_W(tmp_count, tmp_count) = 1000;       // large weight
                    ++tmp_count;
                }
            }

            // ready for solving
            Eigen::VectorXd dx = -(good_G.transpose()*good_W*good_G).inverse() * good_G.transpose() * good_W * good_b;
            dx_norm = dx.norm();
            xyzt += dx;
            // LOG(INFO) << "cov is \n" << (G.transpose()*W*G).inverse();
            ++num_iter;
        }
        if (num_iter == MAX_ITER_PVT)
        {
            LOG(WARNING) << "[gnss_comm::psr_pos] XYZT solver reached maximum iterations.\n";
            return result;
        }

        result = xyzt;
        return result;
    }

    void dopp_res(const Eigen::Matrix<double, 4, 1> &rcv_state, const Eigen::Vector3d &rcv_ecef,
                  const std::vector<ObsPtr> &obs, const std::vector<SatStatePtr> &all_sv_states, 
                  Eigen::VectorXd &res, Eigen::MatrixXd &J)
    {
        const uint32_t num_sv = all_sv_states.size();
        //clear output
        res = Eigen::VectorXd::Zero(num_sv);
        J = Eigen::MatrixXd::Zero(num_sv, 4);

        for (uint32_t i = 0; i < num_sv; ++i)
        {
            const SatStatePtr &sat_state = all_sv_states[i];
            Eigen::Vector3d unit_rv2sv = (sat_state->pos - rcv_ecef).normalized();
            double sagnac_term = EARTH_OMG_GPS/LIGHT_SPEED*(
                sat_state->vel(0)*rcv_ecef(1)+ sat_state->pos(0)*rcv_state(1,0) - 
                sat_state->vel(1)*rcv_ecef(0) - sat_state->pos(1)*rcv_state(0,0));
            double dopp_estimated = (sat_state->vel - rcv_state.topLeftCorner<3, 1>()).dot(unit_rv2sv) + 
                    rcv_state(3, 0) + sagnac_term - sat_state->ddt*LIGHT_SPEED;
            int l1_idx = -1;
            const double obs_freq = L1_freq(obs[i], &l1_idx);
            if (obs_freq < 0)   continue;
            const double wavelength = LIGHT_SPEED / obs_freq;
            res(i) = dopp_estimated + obs[i]->dopp[l1_idx]*wavelength;
            J.block(i, 0, 1, 3) = -1.0 * unit_rv2sv.transpose();
            J(i, 3) = 1.0;
        }
    }

    Eigen::Matrix<double, 4, 1> dopp_vel(const std::vector<ObsPtr> &obs, 
        const std::vector<EphemBasePtr> &ephems, Eigen::Vector3d &ref_ecef)
    {
        LOG(INFO) << "try to solve Doppler velocity.";
        Eigen::Matrix<double, 4, 1> result;
        result.setZero();

        if (ref_ecef.norm() == 0)
        {
            // reference point not given, try calculate using pseudorange
            std::vector<double> zero_iono_params(8, 0.0);
            Eigen::Matrix<double, 7, 1> psr_result = psr_pos(obs, ephems, zero_iono_params);
            if (psr_result.head<3>().norm() != 0)
            {
                ref_ecef = psr_result.head<3>();
            }
            else
            {
                LOG(ERROR) << "[gnss_comm::dopp_vel] Unable to initialize reference position for Doppler calculation.\n";
                return result;
            }
        }

        std::vector<ObsPtr> valid_obs;
        std::vector<EphemBasePtr> valid_ephems;
        filter_L1(obs, ephems, valid_obs, valid_ephems);

        if (valid_obs.size() < 4)
        {
            LOG(ERROR) << "[gnss_comm::dopp_vel] GNSS observation not enough for velocity calculation.\n";
            return result;
        }

        std::vector<SatStatePtr> all_sat_states = sat_states(valid_obs, valid_ephems);
        // compute azel for all satellite
        std::vector<Eigen::Vector2d> all_sv_azel;
        for (uint32_t i = 0; i < all_sat_states.size(); ++i)
        {
            double azel[2] = {0, 0};
            sat_azel(ref_ecef, all_sat_states[i]->pos, azel);
            all_sv_azel.emplace_back(azel[0], azel[1]);
        }

        Eigen::Matrix<double, 4, 1> xyzt_dot;
        xyzt_dot.setZero();
        double dx_norm = 1.0;
        uint32_t num_iter = 0;
        while(num_iter < MAX_ITER_PVT && dx_norm > EPSILON_PVT)
        {
            Eigen::MatrixXd G;
            Eigen::VectorXd b;
            dopp_res(xyzt_dot, ref_ecef, valid_obs, all_sat_states, b, G);
            
            std::vector<uint32_t> good_idx;
            for (uint32_t i = 0; i < valid_obs.size(); ++i)
            {
                if (G.row(i).norm() <= 0)   continue;       // res not computed
                if (all_sv_azel[i].y() > CUT_OFF_DEGREE/180.0*M_PI)  
                    good_idx.push_back(i);
            }
            const uint32_t good_num = good_idx.size();
            Eigen::MatrixXd good_G = Eigen::MatrixXd::Zero(good_num, 4);
            Eigen::VectorXd good_b = Eigen::VectorXd::Zero(good_num);
            Eigen::MatrixXd good_W = Eigen::MatrixXd::Zero(good_num, good_num);
            for (uint32_t i = 0; i < good_num; ++i)
            {
                const uint32_t origin_idx = good_idx[i];
                good_G.row(i) = G.row(origin_idx);
                good_b(i) = b(origin_idx);
                const double sin_el = sin(all_sv_azel[origin_idx].y());
                double weight = sin_el*sin_el;
                const ObsPtr &this_obs = valid_obs[origin_idx];
                int l1_idx = -1;
                L1_freq(this_obs, &l1_idx);
                LOG_IF(FATAL, l1_idx < 0) << "[gnss_comm::dopp_vel] no L1 observation found.\n";
                if (this_obs->dopp_std[l1_idx] > 0)
                    weight /= (this_obs->dopp_std[l1_idx]/0.256);

                const uint32_t obs_sys = satsys(this_obs->sat, NULL);
                if (obs_sys == SYS_GPS || obs_sys == SYS_BDS)
                    weight /= valid_ephems[origin_idx]->ura-1;
                else if (obs_sys == SYS_GAL)
                    weight /= valid_ephems[origin_idx]->ura-2;
                else if (obs_sys == SYS_GLO)
                    weight /= 2;
                good_W(i, i) = weight;
            }

            // solve
            Eigen::VectorXd dx = -(good_G.transpose()*good_W*good_G).inverse() * good_G.transpose() * good_W * good_b;
            dx_norm = dx.norm();
            xyzt_dot += dx;

            // dx_norm = 0;
            // LOG(INFO) << "cov is \n" << (G.transpose()*W*G).inverse();
            ++num_iter;
        }

        if (num_iter == MAX_ITER_PVT)
            LOG(WARNING) << "[gnss_comm::dopp_vel] XYZT solver reached maximum iterations.\n";
        
        result = xyzt_dot;
        return result;
    }
}