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
@brief Definition of static IMU biases used for calibration
*/

#pragma once

#include <Eigen/Dense>

namespace basalt {

/// @brief Static calibration for accelerometer.
///
/// Calibrates axis scaling and misalignment and has 9 parameters \f$ [b_x,
/// b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6]^T \f$.
/// \f[
/// a_c = \begin{bmatrix} s_1 + 1 & 0 & 0 \\ s_2 & s_4 + 1 & 0 \\ s_3 & s_5 &
/// s_6 + 1 \\  \end{bmatrix} a_r -  \begin{bmatrix} b_x \\ b_y \\ b_z
/// \end{bmatrix}
/// \f] where  \f$ a_c \f$ is a calibrated measurement and \f$ a_r \f$ is a
/// raw measurement. When all elements are zero applying calibration results in
/// Identity mapping.
template <typename Scalar>
class CalibAccelBias {
 public:
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Mat33 = Eigen::Matrix<Scalar, 3, 3>;

  /// @brief Default constructor with zero initialization.
  inline CalibAccelBias() { accel_bias_full.setZero(); }

  /// @brief  Set calibration to random values (used in unit-tests).
  inline void setRandom() {
    accel_bias_full.setRandom();
    accel_bias_full.template head<3>() /= 10;
    accel_bias_full.template tail<6>() /= 100;
  }

  /// @brief Return const vector of parameters.
  /// See detailed description in \ref CalibAccelBias.
  inline const Eigen::Matrix<Scalar, 9, 1>& getParam() const {
    return accel_bias_full;
  }

  /// @brief Return vector of parameters. See detailed description in \ref
  /// CalibAccelBias.
  inline Eigen::Matrix<Scalar, 9, 1>& getParam() { return accel_bias_full; }

  /// @brief Increment the calibration vector
  ///
  /// @param inc increment vector
  inline void operator+=(const Eigen::Matrix<Scalar, 9, 1>& inc) {
    accel_bias_full += inc;
  }

  /// @brief Return bias vector and scale matrix. See detailed description in
  /// \ref CalibAccelBias.
  inline void getBiasAndScale(Vec3& accel_bias, Mat33& accel_scale) const {
    accel_bias = accel_bias_full.template head<3>();

    accel_scale.setZero();
    accel_scale.col(0) = accel_bias_full.template segment<3>(3);
    accel_scale(1, 1) = accel_bias_full(6);
    accel_scale(2, 1) = accel_bias_full(7);
    accel_scale(2, 2) = accel_bias_full(8);
  }

  /// @brief Calibrate the measurement. See detailed description in
  /// \ref CalibAccelBias.
  ///
  /// @param raw_measurement
  /// @return calibrated measurement
  inline Vec3 getCalibrated(const Vec3& raw_measurement) const {
    Vec3 accel_bias;
    Mat33 accel_scale;

    getBiasAndScale(accel_bias, accel_scale);

    return (raw_measurement + accel_scale * raw_measurement - accel_bias);
  }

  /// @brief Invert calibration (used in unit-tests).
  ///
  /// @param calibrated_measurement
  /// @return raw measurement
  inline Vec3 invertCalibration(const Vec3& calibrated_measurement) const {
    Vec3 accel_bias;
    Mat33 accel_scale;

    getBiasAndScale(accel_bias, accel_scale);

    Mat33 accel_scale_inv =
        (Eigen::Matrix3d::Identity() + accel_scale).inverse();

    return accel_scale_inv * (calibrated_measurement + accel_bias);
  }

 private:
  Eigen::Matrix<Scalar, 9, 1> accel_bias_full;
};

/// @brief Static calibration for gyroscope.
///
/// Calibrates rotation, axis scaling and misalignment and has 12 parameters \f$
/// [b_x, b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6, s_7, s_8, s_9]^T \f$. \f[
/// \omega_c = \begin{bmatrix} s_1 + 1 & s_4 & s_7 \\ s_2 & s_5 + 1 & s_8 \\ s_3
/// & s_6 & s_9 +1 \\  \end{bmatrix} \omega_r -  \begin{bmatrix} b_x \\ b_y
/// \\ b_z \end{bmatrix} \f] where  \f$ \omega_c \f$ is a calibrated measurement
/// and \f$ \omega_r \f$ is a raw measurement. When all elements are zero
/// applying calibration results in Identity mapping.
template <typename Scalar>
class CalibGyroBias {
 public:
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Mat33 = Eigen::Matrix<Scalar, 3, 3>;

  /// @brief Default constructor with zero initialization.
  inline CalibGyroBias() { gyro_bias_full.setZero(); }

  /// @brief  Set calibration to random values (used in unit-tests).
  inline void setRandom() {
    gyro_bias_full.setRandom();
    gyro_bias_full.template head<3>() /= 10;
    gyro_bias_full.template tail<9>() /= 100;
  }

  /// @brief Return const vector of parameters.
  /// See detailed description in \ref CalibGyroBias.
  inline const Eigen::Matrix<Scalar, 12, 1>& getParam() const {
    return gyro_bias_full;
  }

  /// @brief Return vector of parameters.
  /// See detailed description in \ref CalibGyroBias.
  inline Eigen::Matrix<Scalar, 12, 1>& getParam() { return gyro_bias_full; }

  /// @brief Increment the calibration vector
  ///
  /// @param inc increment vector
  inline void operator+=(const Eigen::Matrix<Scalar, 12, 1>& inc) {
    gyro_bias_full += inc;
  }

  /// @brief Return bias vector and scale matrix. See detailed description in
  /// \ref CalibGyroBias.
  inline void getBiasAndScale(Vec3& gyro_bias, Mat33& gyro_scale) const {
    gyro_bias = gyro_bias_full.template head<3>();
    gyro_scale.col(0) = gyro_bias_full.template segment<3>(3);
    gyro_scale.col(1) = gyro_bias_full.template segment<3>(6);
    gyro_scale.col(2) = gyro_bias_full.template segment<3>(9);
  }

  /// @brief Calibrate the measurement. See detailed description in
  /// \ref CalibGyroBias.
  ///
  /// @param raw_measurement
  /// @return calibrated measurement
  inline Vec3 getCalibrated(const Vec3& raw_measurement) const {
    Vec3 gyro_bias;
    Mat33 gyro_scale;

    getBiasAndScale(gyro_bias, gyro_scale);

    return (raw_measurement + gyro_scale * raw_measurement - gyro_bias);
  }

  /// @brief Invert calibration (used in unit-tests).
  ///
  /// @param calibrated_measurement
  /// @return raw measurement
  inline Vec3 invertCalibration(const Vec3& calibrated_measurement) const {
    Vec3 gyro_bias;
    Mat33 gyro_scale;

    getBiasAndScale(gyro_bias, gyro_scale);

    Mat33 gyro_scale_inv = (Eigen::Matrix3d::Identity() + gyro_scale).inverse();

    return gyro_scale_inv * (calibrated_measurement + gyro_bias);
  }

 private:
  Eigen::Matrix<Scalar, 12, 1> gyro_bias_full;
};

}  // namespace basalt
