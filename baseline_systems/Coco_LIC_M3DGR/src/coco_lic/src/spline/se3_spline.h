/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@file
@brief Uniform B-spline for SE(3)
*/

#pragma once

#include "assert.h"
#include "rd_spline.h"
#include "so3_spline.h"
#include "spline_segment.h"

#include <array>

namespace cocolic {

/// @brief Uniform B-spline for SE(3) of order N. Internally uses an SO(3) (\ref
/// So3Spline) spline for rotation and 3D Euclidean spline (\ref RdSpline) for
/// translation (split representaion).
///
/// See [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860) for more details.
template <int _N, typename _Scalar = double>
class Se3Spline {
 public:
  std::vector<int64_t> knts;
  // std::deque<int64_t> knts;
  std::vector<Eigen::Matrix4d> blending_mats;
  std::vector<Eigen::Matrix4d> cumu_blending_mats;
  int startIdx;

  static constexpr int N = _N;        ///< Order of the spline.
  static constexpr int DEG = _N - 1;  ///< Degree of the spline.

  using MatN = Eigen::Matrix<_Scalar, _N, _N>;
  using VecN = Eigen::Matrix<_Scalar, _N, 1>;
  using VecNp1 = Eigen::Matrix<_Scalar, _N + 1, 1>;

  using Vec3 = Eigen::Matrix<_Scalar, 3, 1>;
  using Vec6 = Eigen::Matrix<_Scalar, 6, 1>;
  using Vec9 = Eigen::Matrix<_Scalar, 9, 1>;
  using Vec12 = Eigen::Matrix<_Scalar, 12, 1>;

  using Mat3 = Eigen::Matrix<_Scalar, 3, 3>;
  using Mat6 = Eigen::Matrix<_Scalar, 6, 6>;

  using Mat36 = Eigen::Matrix<_Scalar, 3, 6>;
  using Mat39 = Eigen::Matrix<_Scalar, 3, 9>;
  using Mat312 = Eigen::Matrix<_Scalar, 3, 12>;

  using Matrix3Array = std::array<Mat3, N>;
  using Matrix36Array = std::array<Mat36, N>;
  using Matrix6Array = std::array<Mat6, N>;

  using SO3 = Sophus::SO3<_Scalar>;
  using SE3 = Sophus::SE3<_Scalar>;

  using PosJacobianStruct = typename RdSpline<3, N, _Scalar>::JacobianStruct;
  using SO3JacobianStruct = typename So3Spline<N, _Scalar>::JacobianStruct;

  /// @brief Struct to store the accelerometer residual Jacobian with
  /// respect to knots
  struct AccelPosSO3JacobianStruct {
    size_t start_idx;
    std::array<Mat36, N> d_val_d_knot;
  };

  /// @brief Struct to store the pose Jacobian with respect to knots
  struct PosePosSO3JacobianStruct {
    size_t start_idx;
    std::array<Mat6, N> d_val_d_knot;
  };

  /// @brief Constructor with knot interval and start time
  ///
  /// @param[in] time_interval_ns knot time interval in seconds
  /// @param[in] start_time_ns start time of the spline in seconds
  Se3Spline(int64_t time_interval_ns, int64_t start_time_ns = 0)
      : pos_spline(time_interval_ns, start_time_ns),
        so3_spline(time_interval_ns, start_time_ns),
        dt_ns_(time_interval_ns) {
          knts.clear();

          // knts.push_back(-0.3 * S_TO_NS);
          // knts.push_back(-0.2 * S_TO_NS);
          // knts.push_back(-0.1 * S_TO_NS);
          
          knts.push_back(-0.09 * S_TO_NS);
          knts.push_back(-0.06 * S_TO_NS);
          knts.push_back(-0.03 * S_TO_NS);
          
          // for (int i = 0; i < 3; i++) knts.push_back(0.0 * S_TO_NS);
          
          blending_mats.clear();
          cumu_blending_mats.clear();
          startIdx = 0;
  }

  /// @brief Gererate random trajectory
  ///
  /// @param[in] n number of knots to generate
  /// @param[in] static_init if true the first N knots will be the same
  /// resulting in static initial condition
  void genRandomTrajectory(int n, bool static_init = false) {
    so3_spline.genRandomTrajectory(n, static_init);
    pos_spline.genRandomTrajectory(n, static_init);
  }

  // void genSemiRandomTrajectory(int n, bool static_init = false) {
  //   so3_spline.genRandomTrajectory(n, static_init);
  //   pos_spline.genRandomTrajectory(n, static_init);
  // }

  /// @brief Set the knot to particular SE(3) pose
  ///
  /// @param[in] pose SE(3) pose
  /// @param[in] i index of the knot
  void setKnot(const SE3 &pose, int i) {
    so3_spline.getKnot(i) = pose.so3();
    pos_spline.getKnot(i) = pose.translation();
  }

  /// @brief Set the knot to particular Vec3 pose
  ///
  /// @param[in] pos Vec3 pose
  /// @param[in] i index of the knot
  void setKnotPos(const Vec3 pos, int i) { pos_spline.getKnot(i) = pos; }

  /// @brief Set the knot to particular SO3 pose
  ///
  /// @param[in] ori SO3 pose
  /// @param[in] i index of the knot
  void setKnotSO3(const SO3 ori, int i) { so3_spline.getKnot(i) = ori; }

  /// @brief Reset spline to have num_knots initialized at pose
  ///
  /// @param[in] pose SE(3) pose
  /// @param[in] num_knots number of knots to initialize
  void setKnots(const SE3 &pose, int num_knots) {
    so3_spline.resize(num_knots);
    pos_spline.resize(num_knots);

    for (int i = 0; i < num_knots; i++) {
      so3_spline.getKnot(i) = pose.so3();
      pos_spline.getKnot(i) = pose.translation();
    }
  }

  /// @brief Reset spline to the knots from other spline
  ///
  /// @param[in] other spline to copy knots from
  void setKnots(const Se3Spline<N, _Scalar> &other) {
    BASALT_ASSERT(other.dt_ns_ == dt_ns_);
    BASALT_ASSERT(other.pos_spline.getKnots().size() ==
                  other.pos_spline.getKnots().size());

    size_t num_knots = other.pos_spline.getKnots().size();

    so3_spline.resize(num_knots);
    pos_spline.resize(num_knots);

    for (size_t i = 0; i < num_knots; i++) {
      so3_spline.getKnot(i) = other.so3_spline.getKnot(i);
      pos_spline.getKnot(i) = other.pos_spline.getKnot(i);
    }
  }

  /// @brief extend trajectory to time t
  ///
  /// @param[in] time_ns time
  /// @param[in] initial_so3 initial knot of so3_spline
  /// @param[in] initial_pos initial knot of pos_spline
  void extendKnotsTo(int64_t time_ns, const SO3 initial_so3,
                     const Vec3 initial_pos) {
    while ((numKnots() < N) || (maxTimeNs() < time_ns)) {
      so3_spline.knots_push_back(initial_so3);
      pos_spline.knots_push_back(initial_pos);
    }
  }

  /// [nurbs]
  void extendKnotsTo(const SO3 initial_so3, const Vec3 initial_pos) {
    while (numKnots() < N) {
      so3_spline.knots_push_back(initial_so3);
      pos_spline.knots_push_back(initial_pos);
    }
  }

  void extendKnotsTo(int num, const SE3 &initial_knot) {
    for (int i = 0; i < num; i++) {
      knots_push_back(initial_knot);
    }
  }

  /// @brief extend trajectory to time t
  ///
  /// @param[in] time_ns timestamp
  /// @param[in] initial_knot initial knot
  void extendKnotsTo(int64_t time_ns, const SE3 &initial_knot) {
    while ((numKnots() < N) || (maxTimeNs() < time_ns)) {
      knots_push_back(initial_knot);
    }
  }

  /// @brief Add knot to the end of the spline
  ///
  /// @param[in] knot knot to add
  inline void knots_push_back(const SE3 &knot) {
    so3_spline.knots_push_back(knot.so3());
    pos_spline.knots_push_back(knot.translation());
  }

  /// @brief Remove knot from the back of the spline
  inline void knots_pop_back() {
    so3_spline.knots_pop_back();
    pos_spline.knots_pop_back();
  }

  /// @brief Return the first knot of the spline
  ///
  /// @return first knot of the spline
  inline SE3 knots_front() const {
    SE3 res(so3_spline.knots_front(), pos_spline.knots_front());

    return res;
  }

  /// @brief Remove first knot of the spline and increase the start time
  inline void knots_pop_front() {
    so3_spline.knots_pop_front();
    pos_spline.knots_pop_front();

    BASALT_ASSERT(so3_spline.minTimeNs() == pos_spline.minTimeNs());
    BASALT_ASSERT(so3_spline.getKnots().size() == pos_spline.getKnots().size());
  }

  /// @brief Return the last knot of the spline
  ///
  /// @return last knot of the spline
  SE3 getLastKnot() {
    BASALT_ASSERT(so3_spline.getKnots().size() == pos_spline.getKnots().size());

    SE3 res(so3_spline.getKnots().back(), pos_spline.getKnots().back());

    return res;
  }

  /// @brief Return knot with index i
  ///
  /// @param i index of the knot
  /// @return knot
  SE3 getKnot(size_t i) const {
    SE3 res(getKnotSO3(i), getKnotPos(i));
    return res;
  }

  /// @brief Return reference to the SO(3) knot with index i
  ///
  /// @param i index of the knot
  /// @return reference to the SO(3) knot
  inline SO3 &getKnotSO3(size_t i) { return so3_spline.getKnot(i); }

  /// @brief Return const reference to the SO(3) knot with index i
  ///
  /// @param i index of the knot
  /// @return const reference to the SO(3) knot
  inline const SO3 &getKnotSO3(size_t i) const { return so3_spline.getKnot(i); }

  /// @brief Return reference to the position knot with index i
  ///
  /// @param i index of the knot
  /// @return reference to the position knot
  inline Vec3 &getKnotPos(size_t i) { return pos_spline.getKnot(i); }

  /// @brief Return const reference to the position knot with index i
  ///
  /// @param i index of the knot
  /// @return const reference to the position knot
  inline const Vec3 &getKnotPos(size_t i) const {
    return pos_spline.getKnot(i);
  }

  /// @brief Set start time for spline
  ///
  /// @param[in] start_time start time of the spline in seconds
  inline void setStartTime(double timestamp) {
    so3_spline.setStartTime(timestamp);
    pos_spline.setStartTime(timestamp);
  }

  /// @brief Apply increment to the knot
  ///
  /// The incremernt vector consists of translational and rotational parts \f$
  /// [\upsilon, \omega]^T \f$. Given the current pose of the knot \f$ R \in
  /// SO(3), p \in \mathbb{R}^3\f$ the updated pose is: \f{align}{ R' &=
  /// \exp(\omega) R
  /// \\ p' &= p + \upsilon
  /// \f}
  ///  The increment is consistent with \ref
  /// PoseState::applyInc.
  ///
  /// @param[in] i index of the knot
  /// @param[in] inc 6x1 increment vector
  template <typename Derived>
  void applyInc(int i, const Eigen::MatrixBase<Derived> &inc) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

    pos_spline.getKnot(i) += inc.template head<3>();
    so3_spline.getKnot(i) =
        SO3::exp(inc.template tail<3>()) * so3_spline.getKnot(i);
  }

  /// @brief Minimum time represented by spline
  ///
  /// @return minimum time represented by spline in seconds
  int64_t minTimeNs() const {
    BASALT_ASSERT_STREAM(so3_spline.minTimeNs() == pos_spline.minTimeNs(),
                         "so3_spline.minTimeNs() " << so3_spline.minTimeNs()
                                                   << " pos_spline.minTimeNs() "
                                                   << pos_spline.minTimeNs());
    return pos_spline.minTimeNs();
  }

  int64_t maxTimeNs() const {
    BASALT_ASSERT_STREAM(so3_spline.maxTimeNs() == pos_spline.maxTimeNs(),
                         "so3_spline.maxTimeNs() " << so3_spline.maxTimeNs()
                                                   << " pos_spline.maxTimeNs() "
                                                   << pos_spline.maxTimeNs());
    return pos_spline.maxTimeNs();
  }

  /// [nurbs]
  // int64_t maxTimeNsNURBS() const {
  //   return knts.back();
  // }

  int64_t maxTimeNsNURBS() const {
    return max_time_ns;
  }

  void SetMaxTimeNsNURBS(int64_t max_time_ns) {
    this->max_time_ns = max_time_ns;
  }

  /// @brief Number of knots in the spline
  size_t numKnots() const { return pos_spline.getKnots().size(); }

 protected:
  /// @brief Linear acceleration in the world frame.
  ///
  /// @param[in] time_ns time to evaluate linear acceleration in seconds
  inline Vec3 transAccelWorld(int64_t time_ns) const {
    return pos_spline.acceleration(time_ns);
  }

  /// @brief Linear velocity in the world frame.
  ///
  /// @param[in] time_ns time to evaluate linear velocity in seconds
  inline Vec3 transVelWorld(int64_t time_ns) const {
    return pos_spline.velocity(time_ns);
  }

  /// [nurbs]
  inline Vec3 transVelWorldNURBS(const std::pair<int, double>& su,
    double delta_t, const Eigen::Matrix4d& blend_mat) const {
    return pos_spline.velocityNURBS(su, delta_t, blend_mat);
  }

  /// @brief Position in the world frame.
  ///
  /// @param[in] time_ns time to evaluate position in seconds
  inline Vec3 positionWorld(int64_t time_ns) const {
    return pos_spline.evaluate(time_ns);
  }

  /// [nurbs]
  inline Vec3 positionWorldNURBS(int64_t time_ns) const {
    return poseNsNURBS(time_ns).translation();
  }

  /// @brief Rotational velocity in the body frame.
  ///
  /// @param[in] time_ns time to evaluate rotational velocity in seconds
  inline Vec3 rotVelBody(int64_t time_ns) const {
    return so3_spline.velocityBody(time_ns);
  }

  /// [nurbs]
  inline Vec3 rotVelBodyNURBS(int64_t time_ns) const {
    std::pair<int, double> su;  //i和u
    bool flag = false;
    for (int i = 0; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        su.first = i;
        su.second = 1.0 * (time_ns - knts[i]) / (knts[i + 1] - knts[i]);
        flag = true;
      }
    }
    if (!flag) std::cout << "[poseNsNURBS query wrong]\n";
    // GetIdxT(time_ns, su);

    Eigen::Matrix4d cumulative_blending_matrix = cumu_blending_mats[su.first - 3];

    return so3_spline.velocityBodyNURBS(su.first - 3, su.second, 
                                                                     (knts[su.first + 1] - knts[su.first]) * NS_TO_S, 
                                                                     cumulative_blending_matrix);
  }

  inline Vec3 rotAccelBody(int64_t time_ns) const {
    return so3_spline.accelerationBody(time_ns);
  }

  /// @brief Evaluate pose.
  ///
  /// @param[in] time_ns time to evaluate pose in seconds
  /// @return SE(3) pose at time
  SE3 poseNs(int64_t time_ns) const {
    SE3 res;

    res.so3() = so3_spline.evaluate(time_ns);
    res.translation() = pos_spline.evaluate(time_ns);

    return res;
  }

  SE3 poseNsNURBS(int64_t time_ns) const {
    SE3 res;

    std::pair<int, double> su;  //i-k u
    bool flag = false;
    for (int i = 0; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        su.first = i;
        su.second = 1.0 * (time_ns - knts[i]) / (knts[i + 1] - knts[i]);
        flag = true;
      }
    }
    if (!flag) std::cout << "[poseNsNURBS query wrong]\n";
    // GetIdxT(time_ns, su);

    std::cout << " blending_mats:"<<  blending_mats.size() << std::endl;
    Eigen::Matrix4d blending_matrix = blending_mats[su.first - 3];

    Eigen::Matrix4d cumulative_blending_matrix = cumu_blending_mats[su.first - 3];

    res.so3() = so3_spline.evaluateNURBS(su.first - 3, su.second, 
                                                                     cumulative_blending_matrix);
    res.translation() = pos_spline.evaluateNURBS(su.first - 3, su.second, 
                                                                     (knts[su.first + 1] - knts[su.first]) * NS_TO_S, 
                                                                     blending_matrix);

    return res;
  }

  SE3 poseNsNURBS(int64_t time_ns, int start_idx) const {
    SE3 res;

    std::pair<int, double> su;  //i-k u
    bool flag = false;
    for (int i = start_idx; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        su.first = i;
        su.second = 1.0 * (time_ns - knts[i]) / (knts[i + 1] - knts[i]);
        flag = true;
      }
    }
    if (!flag) std::cout << "[poseNsNURBS query wrong]\n";
    // GetIdxT(time_ns, su);

    Eigen::Matrix4d blending_matrix = blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = cumu_blending_mats[su.first - 3];

    res.so3() = so3_spline.evaluateNURBS(su.first - 3, su.second, 
                                                                     cumulative_blending_matrix);
    res.translation() = pos_spline.evaluateNURBS(su.first - 3, su.second, 
                                                                     (knts[su.first + 1] - knts[su.first]) * NS_TO_S, 
                                                                     blending_matrix);

    return res;
  }

  /// @brief Evaluate pose and compute Jacobian.
  ///
  /// @param[in] time_ns time to evaluate pose inseconds
  /// @param[out] J Jacobian of the pose with respect to knots
  /// @return SE(3) pose at time
  SE3 poseNs(int64_t time_ns, PosePosSO3JacobianStruct *J) const {
    SE3 res;

    typename So3Spline<_N, _Scalar>::JacobianStruct Jr;
    typename RdSpline<3, N, _Scalar>::JacobianStruct Jp;

    res.so3() = so3_spline.evaluate(time_ns, &Jr);
    res.translation() = pos_spline.evaluate(time_ns, &Jp);

    if (J) {
      Eigen::Matrix3d RT = res.so3().inverse().matrix();

      J->start_idx = Jr.start_idx;
      for (int i = 0; i < N; i++) {
        J->d_val_d_knot[i].setZero();
        J->d_val_d_knot[i].template topLeftCorner<3, 3>() =
            RT * Jp.d_val_d_knot[i];
        J->d_val_d_knot[i].template bottomRightCorner<3, 3>() =
            RT * Jr.d_val_d_knot[i];
      }
    }

    return res;
  }

 public:
  /// @brief Print knots for debugging.
  inline void print_knots() const {
    for (size_t i = 0; i < pos_spline.getKnots().size(); i++) {
      std::cout << i << ": p:" << pos_spline.getKnot(i).transpose() << " q: "
                << so3_spline.getKnot(i).unit_quaternion().coeffs().transpose()
                << std::endl;
    }
  }

  /// @brief Print position knots for debugging.
  inline void print_pos_knots() const {
    std::cout << "Pos Knots : " << std::endl;
    for (size_t i = 0; i < pos_spline.getKnots().size(); i++) {
      std::cout << i << " : " << pos_spline.getKnot(i).transpose() << std::endl;
    }
  }

  /// @brief Knot time interval in nanoseconds.
  inline int64_t getDtNs() const { return dt_ns_; }

  inline double getDt() const { return dt_ns_ * NS_TO_S; }

  std::pair<double, size_t> computeTIndexNs(int64_t time_ns) const {
    return pos_spline.computeTIndexNs(time_ns);
  }

  size_t GetCtrlIndex(int64_t time_ns) const {
    return pos_spline.computeTIndexNs(time_ns).second;
  }

  /// [nurbs]
  size_t GetCtrlIndexNURBS(int64_t time_ns) const {
    int res = INT_MAX;
    for (int i = 0; i < knts.size() -1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        res = i;
        break;
      }
    }
    if (res == INT_MAX) std::cout << "[GetCtrlIndexNURBS wrong]\n";
    return res;
  }

  void GetIdxT(int64_t time_ns, std::pair<int, double>& su) {
    bool flag = false;
    // for (int i = 0; i < knts.size() - 1; i++) {
    for (int i = startIdx; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        su.first = i;
        su.second = 1.0 * (time_ns - knts[i]) / (knts[i + 1] - knts[i]);
        flag = true;
        return;
      }
    }
    if (!flag) { 
      std::cout << "[GetIdxT wrong]\n";
      std::cout << "[start_knt] " << knts[startIdx] << std::endl;
      std::cout << "[knt_max] " << knts.back() << std::endl;
      std::cout << "[time_ns] " << time_ns << std::endl;
    }
  }

  void GetIdxT(int64_t time_ns, std::pair<int, double>& su, bool from_start) {
    bool flag = false;
    // for (int i = 0; i < knts.size() - 1; i++) {
    for (int i = 0; i < knts.size() - 1; i++) {
      if (time_ns >= knts[i] && time_ns < knts[i + 1]) {
        su.first = i;
        su.second = 1.0 * (time_ns - knts[i]) / (knts[i + 1] - knts[i]);
        flag = true;
        return;
      }
    }
    if (!flag) std::cout << "[GetIdxT wrong]\n";
  }

  /// [for vector]
  void InitBlendMat() {
    std::cout<<"use InitBlendMat"<< std::endl;
    Eigen::Matrix4d blending_mat = Eigen::Matrix4d::Zero();
    
    double ti = knts[3] * NS_TO_S;
    double ti_minus_2 = knts[1] * NS_TO_S;
    double ti_minus_1 = knts[2] * NS_TO_S;
    double ti_plus_1 = knts[4] * NS_TO_S;
    double ti_plus_2 = knts[5] * NS_TO_S;
    double ti_plus_3 = knts[6] * NS_TO_S;
    blending_mat(0, 0) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_1- ti_minus_1) * (ti_plus_1 - ti_minus_2));
    blending_mat(0, 2) = (ti - ti_minus_1) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(1, 2) = 3 * (ti_plus_1 - ti) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(2, 2) = 3* (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(3, 3) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_3 - ti) * (ti_plus_2 - ti));
    blending_mat(0, 1) = 1- blending_mat(0, 0) - blending_mat(0, 2);
    blending_mat(0, 3) = 0;
    blending_mat(1, 0) = - 3 * blending_mat(0, 0);
    blending_mat(1, 1) = 3 * blending_mat(0, 0) - blending_mat(1, 2);
    blending_mat(1, 3) = 0;
    blending_mat(2, 0) = 3 * blending_mat(0, 0);
    blending_mat(2, 1) = - 3 * blending_mat(0, 0) - blending_mat(2, 2);
    blending_mat(2, 3) = 0;
    blending_mat(3, 0) = - blending_mat(0, 0);
    blending_mat(3, 2) = - blending_mat(2, 2) / 3 - blending_mat(3, 3) - (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti) * (ti_plus_2 - ti_minus_1));
    blending_mat(3, 1) = blending_mat(0, 0) - blending_mat(3, 2) - blending_mat(3, 3);
    blending_mat = (blending_mat.transpose()).eval();

    // blending_mat = Eigen::Matrix4d::Zero();
    // blending_mat << 1.0 / 6, - 3.0 / 6, 3.0 / 6, - 1.0 / 6,
    //                                     4.0 / 6, 0.0, -6.0 / 6, 3.0 / 6,
    //                                     1.0 / 6, 3.0 / 6, 3.0 / 6, - 3.0 / 6,
    //                                     0.0, 0.0, 0.0, 1.0 / 6;

    Eigen::Matrix4d cumu_blending_mat = blending_mat;
    for (int i = 0; i < 4; i++) 
    {
      for (int j = i + 1; j < 4; j++) 
      {
        cumu_blending_mat.row(i) += cumu_blending_mat.row(j);
      }
    }
    blending_mats.push_back(blending_mat);
    cumu_blending_mats.push_back(cumu_blending_mat);
  }

  void AddBlendMat(int offset) {
    Eigen::Matrix4d blending_mat = Eigen::Matrix4d::Zero();
    std::cout<<"use AddBlendMat"<< std::endl;
    
    int cur_idx = knts.size() - 1 - offset;
    LOG(INFO) << "[cur_idx] " << cur_idx;
    double ti = knts[cur_idx] * NS_TO_S;
    double ti_minus_2 = knts[cur_idx - 2] * NS_TO_S;
    double ti_minus_1 = knts[cur_idx - 1] * NS_TO_S;
    double ti_plus_1 = knts[cur_idx + 1] * NS_TO_S;
    double ti_plus_2 = knts[cur_idx + 2] * NS_TO_S;
    double ti_plus_3 = knts[cur_idx + 3] * NS_TO_S;
    blending_mat(0, 0) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_1- ti_minus_1) * (ti_plus_1 - ti_minus_2));
    blending_mat(0, 2) = (ti - ti_minus_1) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(1, 2) = 3 * (ti_plus_1 - ti) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(2, 2) = 3* (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(3, 3) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_3 - ti) * (ti_plus_2 - ti));
    blending_mat(0, 1) = 1- blending_mat(0, 0) - blending_mat(0, 2);
    blending_mat(0, 3) = 0;
    blending_mat(1, 0) = - 3 * blending_mat(0, 0);
    blending_mat(1, 1) = 3 * blending_mat(0, 0) - blending_mat(1, 2);
    blending_mat(1, 3) = 0;
    blending_mat(2, 0) = 3 * blending_mat(0, 0);
    blending_mat(2, 1) = - 3 * blending_mat(0, 0) - blending_mat(2, 2);
    blending_mat(2, 3) = 0;
    blending_mat(3, 0) = - blending_mat(0, 0);
    blending_mat(3, 2) = - blending_mat(2, 2) / 3 - blending_mat(3, 3) - (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti) * (ti_plus_2 - ti_minus_1));
    blending_mat(3, 1) = blending_mat(0, 0) - blending_mat(3, 2) - blending_mat(3, 3);
    blending_mat = (blending_mat.transpose()).eval();

    // blending_mat = Eigen::Matrix4d::Zero();
    // blending_mat << 1.0 / 6, - 3.0 / 6, 3.0 / 6, - 1.0 / 6,
    //                                     4.0 / 6, 0.0, -6.0 / 6, 3.0 / 6,
    //                                     1.0 / 6, 3.0 / 6, 3.0 / 6, - 3.0 / 6,
    //                                     0.0, 0.0, 0.0, 1.0 / 6;

    Eigen::Matrix4d cumu_blending_mat = blending_mat;
    for (int i = 0; i < 4; i++) 
    {
      for (int j = i + 1; j < 4; j++) 
      {
        cumu_blending_mat.row(i) += cumu_blending_mat.row(j);
      }
    }
    blending_mats.push_back(blending_mat);
    cumu_blending_mats.push_back(cumu_blending_mat);
  }

  void AddBlendMatForGen(int cur_idx) {
    Eigen::Matrix4d blending_mat = Eigen::Matrix4d::Zero();
    std::cout<<"use AddBlendMatForGen"<< std::endl;
    
    double ti = knts[cur_idx] * NS_TO_S;
    double ti_minus_2 = knts[cur_idx - 2] * NS_TO_S;
    double ti_minus_1 = knts[cur_idx - 1] * NS_TO_S;
    double ti_plus_1 = knts[cur_idx + 1] * NS_TO_S;
    double ti_plus_2 = knts[cur_idx + 2] * NS_TO_S;
    double ti_plus_3 = knts[cur_idx + 3] * NS_TO_S;
    blending_mat(0, 0) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_1- ti_minus_1) * (ti_plus_1 - ti_minus_2));
    blending_mat(0, 2) = (ti - ti_minus_1) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(1, 2) = 3 * (ti_plus_1 - ti) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(2, 2) = 3* (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
    blending_mat(3, 3) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_3 - ti) * (ti_plus_2 - ti));
    blending_mat(0, 1) = 1- blending_mat(0, 0) - blending_mat(0, 2);
    blending_mat(0, 3) = 0;
    blending_mat(1, 0) = - 3 * blending_mat(0, 0);
    blending_mat(1, 1) = 3 * blending_mat(0, 0) - blending_mat(1, 2);
    blending_mat(1, 3) = 0;
    blending_mat(2, 0) = 3 * blending_mat(0, 0);
    blending_mat(2, 1) = - 3 * blending_mat(0, 0) - blending_mat(2, 2);
    blending_mat(2, 3) = 0;
    blending_mat(3, 0) = - blending_mat(0, 0);
    blending_mat(3, 2) = - blending_mat(2, 2) / 3 - blending_mat(3, 3) - (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti) * (ti_plus_2 - ti_minus_1));
    blending_mat(3, 1) = blending_mat(0, 0) - blending_mat(3, 2) - blending_mat(3, 3);
    blending_mat = (blending_mat.transpose()).eval();

    // blending_mat = Eigen::Matrix4d::Zero();
    // blending_mat << 1.0 / 6, - 3.0 / 6, 3.0 / 6, - 1.0 / 6,
    //                                     4.0 / 6, 0.0, -6.0 / 6, 3.0 / 6,
    //                                     1.0 / 6, 3.0 / 6, 3.0 / 6, - 3.0 / 6,
    //                                     0.0, 0.0, 0.0, 1.0 / 6;

    Eigen::Matrix4d cumu_blending_mat = blending_mat;
    for (int i = 0; i < 4; i++) 
    {
      for (int j = i + 1; j < 4; j++) 
      {
        cumu_blending_mat.row(i) += cumu_blending_mat.row(j);
      }
    }
    blending_mats.push_back(blending_mat);
    cumu_blending_mats.push_back(cumu_blending_mat);
  }

  void CaculateSplineMeta(time_init_t times,
                          SplineMeta<_N> &spline_meta) const {
    int64_t master_dt_ns = getDtNs();
    int64_t master_t0_ns = minTimeNs();
    size_t current_segment_start = 0;
    size_t current_segment_end = 0;  // Negative signals no segment created yet

    // Times are guaranteed to be sorted correctly and t2 >= t1
    for (auto tt : times) {
      std::pair<double, size_t> ui_1, ui_2;
      ui_1 = pos_spline.computeTIndexNs(tt.first);
      ui_2 = pos_spline.computeTIndexNs(tt.second);

      size_t i1 = ui_1.second;
      size_t i2 = ui_2.second;

      // Create new segment, or extend the current one
      if (spline_meta.segments.empty() || i1 > current_segment_end) {
        int64_t segment_t0_ns = master_t0_ns + master_dt_ns * i1;
        spline_meta.segments.push_back(
            SplineSegmentMeta<_N>(segment_t0_ns, master_dt_ns));
        current_segment_start = i1;
      } else {
        i1 = current_segment_end + 1;
      }

      auto &current_segment_meta = spline_meta.segments.back();

      for (size_t i = i1; i < (i2 + N); ++i) {
        current_segment_meta.n += 1;
      }

      current_segment_end = current_segment_start + current_segment_meta.n - 1;
    }  // for times
  }

  void GetCtrlIdxs(const SplineMeta<_N> &spline_meta,
                   std::vector<int> &ctrl_ids) {
    size_t last_ctrl_id = 0;
    for (auto const &seg : spline_meta.segments) {
      size_t start_idx = GetCtrlIndex(seg.t0_ns);
      for (size_t i = 0; i < seg.NumParameters(); i++) {
        if (i + start_idx > last_ctrl_id || (i + start_idx == 0)) {
          ctrl_ids.push_back(start_idx + i);
        }
      }
      last_ctrl_id = ctrl_ids.back();
    }
  }

  void GetCtrlIdxs(const std::vector<double> &timestamps,
                   std::vector<int> &ctrl_ids) {
    int last_ctrl_id = -1;
    for (const double &t : timestamps) {
      int start_idx = GetCtrlIndex(t);
      for (int i = 0; i < SplineOrder; i++) {
        if (i + start_idx > last_ctrl_id) {
          ctrl_ids.push_back(start_idx + i);
        }
      }
      last_ctrl_id = ctrl_ids.back();
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  RdSpline<3, _N, _Scalar> pos_spline;  ///< Position spline
  So3Spline<_N, _Scalar> so3_spline;    ///< Orientation spline

  int64_t dt_ns_;  ///< Knot interval in nanoseconds

  int64_t max_time_ns;
};

}  // namespace cocolic
