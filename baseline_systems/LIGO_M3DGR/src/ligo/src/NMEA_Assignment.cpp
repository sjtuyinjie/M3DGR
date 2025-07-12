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

#include "NMEA_Assignment.h"

NMEAAssignment::NMEAAssignment() : process_feat_num(0) {}

void NMEAAssignment::initNoises( void ) // maybe usable!
{
    gtsam::Vector priorrotNoiseVector3(3);
    priorrotNoiseVector3 << prior_noise, prior_noise, prior_noise;
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
    priorNoiseVector6 << prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector priorNoiseVector12(12);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector12 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorBiasNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector12);

    gtsam::Vector priorNoiseVector3(3);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector3 << marg_noise, marg_noise, marg_noise; //, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorGravNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector3);

    gtsam::Vector margrotNoiseVector3(3);
    margrotNoiseVector3 << prior_noise / 10, prior_noise / 10, prior_noise / 10;
    margrotNoise = gtsam::noiseModel::Diagonal::Variances(margrotNoiseVector3);

    gtsam::Vector margposNoiseVector6(6);
    margposNoiseVector6 << prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise; //, marg_noise, marg_noise, marg_noise; //, marg_noise, marg_noise, marg_noise;
    margposNoise = gtsam::noiseModel::Diagonal::Variances(margposNoiseVector6);

    gtsam::Vector odomaNoiseVector12(12);
    odomaNoiseVector12 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise;
    odomaNoise = gtsam::noiseModel::Diagonal::Variances(odomaNoiseVector12);

    gtsam::Vector priorextrotNoiseVector3(3);
    priorextrotNoiseVector3 << prior_noise, prior_noise, prior_noise;
    priorextrotNoise = gtsam::noiseModel::Diagonal::Variances(priorextrotNoiseVector3);

    gtsam::Vector margNoiseVector3(3);
    margNoiseVector3 << prior_noise / 10, prior_noise / 10, prior_noise / 10; //, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, 
                        // marg_noise, marg_noise;
    margNoise = gtsam::noiseModel::Diagonal::Variances(margNoiseVector3);

    gtsam::Vector priorextposNoiseVector3(3);
    priorextposNoiseVector3 << prior_noise, prior_noise, prior_noise;
    priorextposNoise = gtsam::noiseModel::Diagonal::Variances(priorextposNoiseVector3);

    gtsam::Vector odomNoiseVector27(27);
    odomNoiseVector27 << grav_noise, grav_noise, grav_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, 
                          marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise;
    odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector27); // should be related to the time, maybe proportional

    gtsam::Vector odomNoiseVector15(15);
    odomNoiseVector15 << odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise,
                        odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise;
    odomNoiseIMU = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector15); // should be related to the imu noise
    // odomNoiseIMU = gtsam::noiseModel::Robust::Create(
    //                 gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                 gtsam::noiseModel::Diagonal::Variances(odomNoiseVector15));
    gtsam::Vector relatNoiseVector9(9);
    relatNoiseVector9 << odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise;
    relatNoise = gtsam::noiseModel::Diagonal::Variances(relatNoiseVector9);
    gtsam::Vector robustnmeaNoiseVector9(3); // gtsam::Pose3 factor has 6 elements (6D)
    robustnmeaNoiseVector9 << pos_noise, pos_noise, pos_noise; //, vel_noise, vel_noise, vel_noise, rot_noise, rot_noise, rot_noise;
    if (outlier_rej)
    {
      robustnmeaNoise = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustnmeaNoiseVector9));
      
      robustnmeaNoise_init = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustnmeaNoiseVector9));
    }
    else
    {
      robustnmeaNoise = gtsam::noiseModel::Diagonal::Variances(robustnmeaNoiseVector9);
      robustnmeaNoise_init = gtsam::noiseModel::Diagonal::Variances(robustnmeaNoiseVector9);
    }
} // initNoises

void NMEAAssignment::delete_variables(bool nolidar, size_t frame_delete, int frame_num, size_t &id_accumulate, gtsam::FactorIndices delete_factor)
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

            gtsam::PriorFactor<gtsam::Rot3> init_rot(R(frame_delete+j),isamCurrentEstimate.at<gtsam::Rot3>(R(frame_delete+j)), margrotNoise); // updatedRotNoise); //  
            // gtsam::PriorFactor<gtsam::Vector12> init_vel(F(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector12>(F(frame_delete+j)), updatedPosNoise); // margposNoise);
            gtsam::PriorFactor<gtsam::Vector6> init_vel(A(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector6>(A(frame_delete+j)), margposNoise); // updatedPosNoise); // 
            gtSAMgraph.add(init_rot);
            gtSAMgraph.add(init_vel);
            factor_id_frame[0].push_back(id_accumulate+(j)*2);
            factor_id_frame[0].push_back(id_accumulate+1+(j)*2);
        }
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

            gtsam::PriorFactor<gtsam::Rot3> init_rot(R(frame_delete+j),isamCurrentEstimate.at<gtsam::Rot3>(R(frame_delete+j)), updatedRotNoise); // margrotNoise);
            gtsam::PriorFactor<gtsam::Vector12> init_vel(F(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector12>(F(frame_delete+j)), updatedPosNoise); // margposNoise);
            gtSAMgraph.add(init_rot);
            gtSAMgraph.add(init_vel);
            {
            factor_id_frame[0].push_back(id_accumulate+j*2);
            factor_id_frame[0].push_back(id_accumulate+1+j*2);
            }
        }
        id_accumulate += j * 2;
      }
      isam.update(gtSAMgraph, initialEstimate);
      gtSAMgraph.resize(0); // will the initialEstimate change?
      initialEstimate.clear();
      isam.update(gtSAMgraph, initialEstimate, delete_factor);
    }
}  