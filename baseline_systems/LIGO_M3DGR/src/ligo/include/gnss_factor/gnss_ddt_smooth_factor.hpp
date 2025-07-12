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

#ifndef DDT_SMOOTH_FACTOR_H_
#define DDT_SMOOTH_FACTOR_H_

#include <Eigen/Dense>

// #include <gtsam/geometry/Rot3.h>
// #include <gtsam/geometry/Pose3.h>
// #include <gtsam/slam/PriorFactor.h>
// #include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
using namespace gnss_comm;

namespace ligo {

class DdtSmoothFactor : public gtsam::NoiseModelFactor2<gtsam::Vector1, gtsam::Vector1>
{
    public: 
        DdtSmoothFactor(gtsam::Key i, gtsam::Key j, const gtsam::SharedNoiseModel& model) : 
        gtsam::NoiseModelFactor2<gtsam::Vector1, gtsam::Vector1>(model, i, j) {
        }
        virtual ~DdtSmoothFactor() {}
        gtsam::Vector evaluateError(const gtsam::Vector1 &ddti, const gtsam::Vector1 &ddtj, boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const
        {
            if (H1) //(*H1) = (gtsam::Matrix(1,1)<<-1.0).finished();
            {
                (*H1) = - gtsam::Matrix::Identity(1, 1);
            }
            if (H2) //(*H2) = (gtsam::Matrix(1,1)<<1.0).finished();
            {
                (*H2) = gtsam::Matrix::Identity(1, 1);
            }
            gtsam::Vector1 residual;
            residual(0) = ddtj[0] - ddti[0];
            return residual;
            // return (gtsam::Vector(1) << ddtj - ddti).finished();
        }
    // private:
        // double weight_;
};
} 

#endif