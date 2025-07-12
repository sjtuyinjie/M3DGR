/*
 * Coco-LIC: Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ceres/ceres.h>
#include <spline/spline_segment.h>
#include <utils/parameter_struct.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <odom/factor/analytic_diff/split_spline_view.h>

namespace cocolic
{
  namespace analytic_derivative
  {

    class PnPFactorNURBS : public ceres::CostFunction,
                           So3SplineView,
                           RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;
      using SplitView = SplitSpineView;

      using Vec2d = Eigen::Matrix<double, 2, 1>;
      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using Mat3d = Eigen::Matrix<double, 3, 3>;
      using SO3d = Sophus::SO3<double>;

      PnPFactorNURBS(
          const int64_t t_img, const std::pair<int, double> &su,
          const Eigen::Matrix4d &blending_matrix, const Eigen::Matrix4d &cumulative_blending_matrix,
          const Eigen::Vector3d &v_point, const Eigen::Vector2d &px_obs,
          const SO3d &S_VtoI, const Eigen::Vector3d &p_VinI, const Eigen::Matrix3d &K,
          double img_weight)
          : t_img_(t_img),
            su_(su),
            blending_matrix_(blending_matrix),
            cumulative_blending_matrix_(cumulative_blending_matrix),
            v_point_(v_point), // 
            px_obs_(px_obs),
            S_VtoI_(S_VtoI), p_VinI_(p_VinI), K_(K),
            img_weight_(img_weight)
      {
        /// 
        set_num_residuals(2);
        /// 
        size_t kont_num = 4;
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        SO3d S_ItoG;
        Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();

        if (jacobians)
        {
          SplitView::EvaluatePhotoRpNURBS(su_, cumulative_blending_matrix_,
                                          blending_matrix_, parameters,
                                          S_VtoI_, p_VinI_, v_point_,
                                          &S_ItoG, &p_IinG, &J_R, &J_p);
        }
        else
        {
          SplitView::EvaluatePhotoRpNURBS(su_, cumulative_blending_matrix_,
                                          blending_matrix_, parameters,
                                          S_VtoI_, p_VinI_, v_point_,
                                          &S_ItoG, &p_IinG, nullptr, nullptr);
        }

        // Vec3d p_C = S_VtoI_.inverse() * S_ItoG.inverse() * v_point_ - S_VtoI_.inverse() * S_ItoG.inverse() * p_IinG - S_VtoI_.inverse() * p_VinI_;
        SE3d Twb(S_ItoG.unit_quaternion().toRotationMatrix(), p_IinG);
        SE3d Tbc(S_VtoI_.unit_quaternion().toRotationMatrix(), p_VinI_);
        SE3d Twc = Twb * Tbc;
        Vec3d p_C = Twc.inverse() * v_point_; 

        double fx = K_(0, 0);
        double cx = K_(0, 2);
        double fy = K_(1, 1);
        double cy = K_(1, 2);
        Vec2d uv;
        uv << fx * p_C.x() / p_C.z() + cx, fy * p_C.y() / p_C.z() + cy;

        residuals[0] = (px_obs_.x() - uv.x()) * img_weight_;
        residuals[1] = (px_obs_.y() - uv.y()) * img_weight_;

        if (!jacobians)
        {
          return true;
        }

        if (jacobians)
        {
          for (size_t i = 0; i < 4; ++i)
          {
            if (jacobians[i])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i]);
              jac_kont_R.setZero();
            }
            if (jacobians[i + 4])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + 4]);
              jac_kont_p.setZero();
            }
          }
        }

        // 2x3
        Eigen::Matrix<double, 2, 3> d_uv_d_pC;
        double X = p_C.x(), Y = p_C.y(), Z = p_C.z();
        // d_uv_d_pC << fx / Z, 0, -fx * X / (Z * Z),
        //     0, fy / Z, -fy * Y / (Z * Z);
        d_uv_d_pC << - fx / Z, 0, fx * X / (Z * Z),
            0, - fy / Z, fy * Y / (Z * Z);

        //
        Eigen::Matrix3d d_pC_d_twb = -S_VtoI_.inverse().matrix() * S_ItoG.inverse().matrix();

        ///
        for (size_t i = 0; i < 4; i++)
        {
          size_t idx = i;
          if (jacobians[idx])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            jac_kont_R.setZero();
            jac_kont_R.block(0, 0, 2, 3) = d_uv_d_pC * J_R.d_val_d_knot[i]; // J_R为d_pC_d_knot
            jac_kont_R = (img_weight_ * jac_kont_R).eval();
          }
        }

        /// 
        for (size_t i = 0; i < 4; i++)
        {
          size_t idx = 4 + i;
          if (jacobians[idx])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                jacobians[idx]);
            jac_kont_p.setZero();
            jac_kont_p = d_uv_d_pC * d_pC_d_twb * J_p.d_val_d_knot[i]; // J_p为d_twb_d_knot
            jac_kont_p = (img_weight_ * jac_kont_p).eval();
          }
        }

        return true;
      }

    private:
      int64_t t_img_;
      std::pair<int, double> su_;
      Eigen::Matrix4d blending_matrix_;
      Eigen::Matrix4d cumulative_blending_matrix_;
      Eigen::Vector3d v_point_;
      Eigen::Vector2d px_obs_;
      SO3d S_VtoI_;
      Eigen::Vector3d p_VinI_;
      Eigen::Matrix3d K_;
      double img_weight_;
    };

    class PhotometricFactorNURBS : public ceres::CostFunction,
                                   So3SplineView,
                                   RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;
      using SplitView = SplitSpineView;

      using Vec2d = Eigen::Matrix<double, 2, 1>;
      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using Mat3d = Eigen::Matrix<double, 3, 3>;
      using SO3d = Sophus::SO3<double>;

      PhotometricFactorNURBS(
          const int patch_size_half, const int scale, int level,
          const int64_t t_img, const std::pair<int, double> &su,
          const Eigen::Matrix4d &blending_matrix, const Eigen::Matrix4d &cumulative_blending_matrix,
          //    const double &prev_pixel_intensity,
          float *patch,
          const Eigen::Vector3d &v_point, const cv::Mat &cur_img,
          const SO3d &S_VtoI, const Eigen::Vector3d &p_VinI, const Eigen::Matrix3d &K,
          double img_weight)
          : patch_size_half_(patch_size_half),
            patch_size_(2 * patch_size_half),
            scale_(scale),
            level_(level),
            //
            t_img_(t_img),
            su_(su),
            blending_matrix_(blending_matrix),
            cumulative_blending_matrix_(cumulative_blending_matrix),
            //   prev_pixel_intensity_(prev_pixel_intensity),
            patch_(patch),
            v_point_(v_point), // 
            cur_img_(cur_img),
            S_VtoI_(S_VtoI), p_VinI_(p_VinI), K_(K),
            img_weight_(img_weight)
      {
        ///
        set_num_residuals(patch_size_ * patch_size_);
        ///
        size_t kont_num = 4;
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        SO3d S_ItoG;
        Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();

        if (jacobians)
        {
          SplitView::EvaluatePhotoRpNURBS(su_, cumulative_blending_matrix_,
                                          blending_matrix_, parameters,
                                          S_VtoI_, p_VinI_, v_point_,
                                          &S_ItoG, &p_IinG, &J_R, &J_p);
        }
        else
        {
          SplitView::EvaluatePhotoRpNURBS(su_, cumulative_blending_matrix_,
                                          blending_matrix_, parameters,
                                          S_VtoI_, p_VinI_, v_point_,
                                          &S_ItoG, &p_IinG, nullptr, nullptr);
        }

        // Vec3d p_C = S_VtoI_.inverse() * S_ItoG.inverse() * v_point_ - S_VtoI_.inverse() * S_ItoG.inverse() * p_IinG - S_VtoI_.inverse() * p_VinI_;
        SE3d Twb(S_ItoG.unit_quaternion().toRotationMatrix(), p_IinG);
        SE3d Tbc(S_VtoI_.unit_quaternion().toRotationMatrix(), p_VinI_);
        SE3d Twc = Twb * Tbc;
        Vec3d p_C = Twc.inverse() * v_point_; //

        //
        double fx = K_(0, 0);
        double cx = K_(0, 2);
        double fy = K_(1, 1);
        double cy = K_(1, 2);
        Vec2d uv;
        //
        uv << fx * p_C.x() / p_C.z() + cx, fy * p_C.y() / p_C.z() + cy;
        double u = uv[0];
        double v = uv[1];
        // 
        int u_i = std::floor(u / scale_) * scale_; // 
        int v_i = std::floor(v / scale_) * scale_;
        double sub_u = (u - u_i) / scale_; //
        double sub_v = (v - v_i) / scale_;
        double w_tl = (1.0 - sub_u) * (1.0 - sub_v); // 
        double w_tr = sub_u * (1.0 - sub_v);
        double w_bl = (1.0 - sub_u) * sub_v;
        double w_br = sub_u * sub_v;

        int patch_size_total = patch_size_ * patch_size_;
        int res_idx = 0;
        Eigen::MatrixXd d_res_d_uv(patch_size_ * patch_size_, 2); // Nx2
        for (int x = 0; x < patch_size_; x++)
        {
          uint8_t *img_ptr =
              (uint8_t *)cur_img_.data + (v_i + x * scale_ - patch_size_half_ * scale_) * cur_img_.cols + u_i - patch_size_half_ * scale_;
          for (int y = 0; y < patch_size_; ++y, img_ptr += scale_)
          {
            double res = patch_[patch_size_total * level_ + x * patch_size_ + y] -
                         (w_tl * img_ptr[0] + w_tr * img_ptr[scale_] +
                          w_bl * img_ptr[scale_ * cur_img_.cols] + w_br * img_ptr[scale_ * cur_img_.cols + scale_]);
            double du = 0.5 * ((w_tl * img_ptr[scale_] + w_tr * img_ptr[scale_ * 2] + w_bl * img_ptr[scale_ * cur_img_.cols + scale_] + w_br * img_ptr[scale_ * cur_img_.cols + scale_ * 2]) - (w_tl * img_ptr[-scale_] + w_tr * img_ptr[0] + w_bl * img_ptr[scale_ * cur_img_.cols - scale_] + w_br * img_ptr[scale_ * cur_img_.cols])) * (1.0 / scale_);
            double dv = 0.5 *
                        ((w_tl * img_ptr[scale_ * cur_img_.cols] + w_tr * img_ptr[scale_ + scale_ * cur_img_.cols] +
                          w_bl * img_ptr[cur_img_.cols * scale_ * 2] +
                          w_br * img_ptr[cur_img_.cols * scale_ * 2 + scale_]) -
                         (w_tl * img_ptr[-scale_ * cur_img_.cols] + w_tr * img_ptr[-scale_ * cur_img_.cols + scale_] +
                          w_bl * img_ptr[0] + w_br * img_ptr[scale_])) *
                        (1.0 / scale_);
            d_res_d_uv(res_idx, 0) = -du;
            d_res_d_uv(res_idx, 1) = -dv;
            residuals[res_idx] = res;
            residuals[res_idx] *= img_weight_;
            res_idx++;
          }
        }

        if (!jacobians)
        {
          return true;
        }

        if (jacobians)
        {
          for (size_t i = 0; i < 4; ++i)
          {
            if (jacobians[i])
            {
              // Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              //     jacobians[i]);
              // Eigen::Map<Eigen::Matrix<double, 64, 4, Eigen::RowMajor>> jac_kont_R(
              //     jacobians[i]);
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i], patch_size_ * patch_size_, 4);
              jac_kont_R.setZero();
            }
            if (jacobians[i + 4])
            {
              // Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              //     jacobians[i + 4]);
              // Eigen::Map<Eigen::Matrix<double, 64, 3, Eigen::RowMajor>> jac_kont_p(
              //     jacobians[i + 4]);
              Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + 4], patch_size_ * patch_size_, 3);
              jac_kont_p.setZero();
            }
          }
        }


        // 2x3
        Eigen::Matrix<double, 2, 3> d_uv_d_pC;
        double X = p_C.x(), Y = p_C.y(), Z = p_C.z();
        d_uv_d_pC << fx / Z, 0, -fx * X / (Z * Z),
            0, fy / Z, -fy * Y / (Z * Z);

        //
        Eigen::Matrix3d d_pC_d_twb = -S_VtoI_.inverse().matrix() * S_ItoG.inverse().matrix();

        /// Rotation control point jacobian
        for (size_t i = 0; i < 4; i++)
        {
          size_t idx = i;
          if (jacobians[idx])
          {
            // Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            //     jacobians[idx]);
            // Eigen::Map<Eigen::Matrix<double, 64, 4, Eigen::RowMajor>> jac_kont_R(
            //     jacobians[idx]);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx], patch_size_ * patch_size_, 4);
            jac_kont_R.setZero();

            // jac_kont_R.block<1, 3>(0, 0) = d_res_d_uv * d_uv_d_pC * J_R.d_val_d_knot[i]; //J_R为d_pC_d_knot
            // jac_kont_R.block<64, 3>(0, 0) = d_res_d_uv * d_uv_d_pC * J_R.d_val_d_knot[i]; //J_R为d_pC_d_knot
            // jac_kont_R.block<patch_size_*patch_size_, 3>(0, 0) = d_res_d_uv * d_uv_d_pC * J_R.d_val_d_knot[i]; //J_R为d_pC_d_knot
            jac_kont_R.block(0, 0, patch_size_ * patch_size_, 3) = d_res_d_uv * d_uv_d_pC * J_R.d_val_d_knot[i]; // J_R为d_pC_d_knot
            //
            jac_kont_R = (img_weight_ * jac_kont_R).eval();
          }
        }

        /// Position control point jacobian
        for (size_t i = 0; i < 4; i++)
        {
          size_t idx = 4 + i;
          if (jacobians[idx])
          {
            // Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            //     jacobians[idx]);
            // Eigen::Map<Eigen::Matrix<double, 64, 3, Eigen::RowMajor>> jac_kont_p(
            //     jacobians[idx]);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac_kont_p(
                jacobians[idx], patch_size_ * patch_size_, 3);
            jac_kont_p.setZero();

            jac_kont_p = d_res_d_uv * d_uv_d_pC * d_pC_d_twb * J_p.d_val_d_knot[i]; // J_p为d_twb_d_knot
            jac_kont_p = (img_weight_ * jac_kont_p).eval();
          }
        }

        return true;
      }

    private:
      inline double GetPixelValue(double u, double v) const
      {
        uchar *data = &cur_img_.data[int(v) * cur_img_.step + int(u)];
        double uu = u - std::floor(u);
        double vv = v - std::floor(v);
        return double(
            (1 - uu) * (1 - vv) * data[0] +
            uu * (1 - vv) * data[1] +
            (1 - uu) * vv * data[cur_img_.step] +
            uu * vv * data[cur_img_.step + 1]);
      }

      inline double GetPyramidIntPixelValue(int u_i, int v_i, double w_tl, double w_tr, double w_bl, double w_br) const
      {
        // 
        uchar *data = &cur_img_.data[v_i * cur_img_.step + u_i];

        return double(w_tl * data[0] +
                      w_tr * data[scale_] +
                      w_bl * data[scale_ * cur_img_.step] +
                      w_br * data[scale_ * cur_img_.step + scale_]);
      }

      int patch_size_half_;
      int patch_size_;
      int scale_; // 
      int level_;
      int64_t t_img_;
      std::pair<int, double> su_;
      Eigen::Matrix4d blending_matrix_;
      Eigen::Matrix4d cumulative_blending_matrix_;
      // double prev_pixel_intensity_;
      float *patch_;
      Eigen::Vector3d v_point_;
      cv::Mat cur_img_;
      SO3d S_VtoI_;
      Eigen::Vector3d p_VinI_;
      Eigen::Matrix3d K_;
      double img_weight_;
    };

    /// 
    class PhotometricFactorNURBS2 : public ceres::CostFunction,
                                    So3SplineView,
                                    RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;
      using SplitView = SplitSpineView;

      using Vec2d = Eigen::Matrix<double, 2, 1>;
      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using Mat3d = Eigen::Matrix<double, 3, 3>;
      using SO3d = Sophus::SO3<double>;

      PhotometricFactorNURBS2(const int64_t t_img, const std::pair<int, double> &su,
                              const Eigen::Matrix4d &blending_matrix, const Eigen::Matrix4d &cumulative_blending_matrix,
                              const double &prev_pixel_intensity, const Eigen::Vector3d &v_point, const cv::Mat &cur_img,
                              const SO3d &S_VtoI, const Eigen::Vector3d &p_VinI, const Eigen::Matrix3d &K,
                              double img_weight)
          : t_img_(t_img),
            su_(su),
            blending_matrix_(blending_matrix),
            cumulative_blending_matrix_(cumulative_blending_matrix),
            prev_pixel_intensity_(prev_pixel_intensity),
            v_point_(v_point),
            cur_img_(cur_img),
            S_VtoI_(S_VtoI), p_VinI_(p_VinI), K_(K),
            img_weight_(img_weight)
      {
        /// 
        set_num_residuals(2);
        /// 
        size_t kont_num = 4;
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        SO3d S_ItoG;
        Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();

        if (jacobians)
        {
          SplitView::EvaluatePhotoRpNURBS(su_, cumulative_blending_matrix_,
                                          blending_matrix_, parameters,
                                          S_VtoI_, p_VinI_, v_point_,
                                          &S_ItoG, &p_IinG, &J_R, &J_p);
        }
        else
        {
          SplitView::EvaluatePhotoRpNURBS(su_, cumulative_blending_matrix_,
                                          blending_matrix_, parameters,
                                          S_VtoI_, p_VinI_, v_point_,
                                          &S_ItoG, &p_IinG, nullptr, nullptr);
        }

        // Vec3d p_C = S_VtoI_.inverse() * S_ItoG.inverse() * v_point_ - S_VtoI_.inverse() * S_ItoG.inverse() * p_IinG - S_VtoI_.inverse() * p_VinI_;
        SE3d Twb(S_ItoG.unit_quaternion().toRotationMatrix(), p_IinG);
        SE3d Tbc(S_VtoI_.unit_quaternion().toRotationMatrix(), p_VinI_);
        SE3d Twc = Twb * Tbc;
        Vec3d p_C = Twc.inverse() * v_point_;

        double fx = K_(0, 0);
        double cx = K_(0, 2);
        double fy = K_(1, 1);
        double cy = K_(1, 2);
        Vec2d uv;
        uv << fx * p_C.x() / p_C.z() + cx, fy * p_C.y() / p_C.z() + cy;
        residuals[0] = uv(0, 0);
        residuals[1] = uv(1, 0);

        if (!jacobians)
        {
          return true;
        }

        if (jacobians)
        {
          for (size_t i = 0; i < 4; ++i)
          {
            if (jacobians[i])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i]);
              jac_kont_R.setZero();
            }
            if (jacobians[i + 4])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + 4]);
              jac_kont_p.setZero();
            }
          }
        }

        Eigen::Matrix<double, 2, 3> d_uv_d_pC;
        double X = p_C.x(), Y = p_C.y(), Z = p_C.z();
        d_uv_d_pC << fx / Z, 0, -fx * X / (Z * Z),
            0, fy / Z, -fy * Y / (Z * Z);
        Eigen::Matrix3d d_pC_d_twb = -S_VtoI_.inverse().matrix() * S_ItoG.inverse().matrix();

        /// Rotation control point jacobian
        for (size_t i = 0; i < 4; i++)
        {
          size_t idx = i;
          if (jacobians[idx])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            jac_kont_R.setZero();

            jac_kont_R.block<2, 3>(0, 0) = d_uv_d_pC * J_R.d_val_d_knot[i];
          }
        }

        /// Position control point jacobian
        for (size_t i = 0; i < 4; i++)
        {
          size_t idx = 4 + i;
          if (jacobians[idx])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                jacobians[idx]);
            jac_kont_p.setZero();

            jac_kont_p = d_uv_d_pC * d_pC_d_twb * J_p.d_val_d_knot[i];
          }
        }

        return true;
      }

    private:
      int64_t t_img_;
      std::pair<int, double> su_;
      Eigen::Matrix4d blending_matrix_;
      Eigen::Matrix4d cumulative_blending_matrix_;
      double prev_pixel_intensity_;
      Eigen::Vector3d v_point_;
      cv::Mat cur_img_;
      SO3d S_VtoI_;
      Eigen::Vector3d p_VinI_;
      Eigen::Matrix3d K_;
      double img_weight_;
    };

    class ImageFeatureFactor : public ceres::CostFunction,
                               So3SplineView,
                               RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;

      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using Mat3d = Eigen::Matrix<double, 3, 3>;
      using SO3d = Sophus::SO3<double>;

      ImageFeatureFactor(const int64_t t_i_ns, const Eigen::Vector3d &p_i,
                         const int64_t t_j_ns, const Eigen::Vector3d &p_j,
                         const SplineMeta<SplineOrder> &spline_meta)
          : t_i_ns_(t_i_ns),
            p_i_(p_i),
            t_j_ns_(t_j_ns),
            p_j_(p_j),
            spline_meta_(spline_meta)
      {
        assert(init_flag && "ImageFeatureFactor not init param");

        /// 
        set_num_residuals(2);

        /// 
        size_t kont_num = spline_meta.NumParameters();
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
        mutable_parameter_block_sizes()->push_back(1); // inverse depth
        mutable_parameter_block_sizes()->push_back(1); // time offset
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R[2];
        typename R3View::JacobianStruct J_p[2];

        size_t Knot_offset = 2 * spline_meta_.NumParameters();
        double d_inv = parameters[Knot_offset][0];
        double time_offset_in_ns = parameters[Knot_offset + 1][0];
        int64_t ti_corrected_ns = t_i_ns_ + (int64_t)time_offset_in_ns;
        int64_t tj_corrected_ns = t_j_ns_ + (int64_t)time_offset_in_ns;

        size_t kont_num = spline_meta_.NumParameters();

        size_t R_offset[2] = {0, 0};
        size_t P_offset[2] = {0, 0};
        size_t seg_idx[2] = {0, 0};
        {
          double u;
          spline_meta_.ComputeSplineIndex(ti_corrected_ns, R_offset[0], u);
          spline_meta_.ComputeSplineIndex(tj_corrected_ns, R_offset[1], u);

          // 
          size_t segment0_knot_num = spline_meta_.segments.at(0).NumParameters();
          for (int i = 0; i < 2; ++i)
          {
            if (R_offset[i] >= segment0_knot_num)
            {
              seg_idx[i] = 1;
              R_offset[i] = segment0_knot_num;
            }
            else
            {
              R_offset[i] = 0;
            }
            P_offset[i] = R_offset[i] + kont_num;
          }
        }

        Vec3d x_ci = p_i_ / d_inv;
        /// 
        Vec3d p_Ii = S_CtoI * x_ci + p_CinI;

        SO3d S_IitoG;
        Vec3d p_IiinG = Vec3d::Zero();
        if (jacobians)
        {
          // rhs = p_Ii
          S_IitoG = SO3View::EvaluateRp(ti_corrected_ns,
                                        spline_meta_.segments.at(seg_idx[0]),
                                        parameters + R_offset[0], &J_R[0]);
          p_IiinG = R3View::evaluate(ti_corrected_ns,
                                     spline_meta_.segments.at(seg_idx[0]),
                                     parameters + P_offset[0], &J_p[0]);
        }
        else
        {
          S_IitoG = SO3View::EvaluateRp(ti_corrected_ns,
                                        spline_meta_.segments.at(seg_idx[0]),
                                        parameters + R_offset[0], nullptr);
          p_IiinG = R3View::evaluate(ti_corrected_ns,
                                     spline_meta_.segments.at(seg_idx[0]),
                                     parameters + P_offset[0], nullptr);
        }
        /// 
        Vec3d p_G = S_IitoG * p_Ii + p_IiinG;
        SO3d S_GtoIj;
        Vec3d p_IjinG = Vec3d::Zero();
        if (jacobians)
        {
          // rhs = p_G - p_IjinG
          S_GtoIj = SO3View::EvaluateRTp(tj_corrected_ns,
                                         spline_meta_.segments.at(seg_idx[1]),
                                         parameters + R_offset[1], &J_R[1]);
          p_IjinG = R3View::evaluate(tj_corrected_ns,
                                     spline_meta_.segments.at(seg_idx[1]),
                                     parameters + P_offset[1], &J_p[1]);
        }
        else
        {
          S_GtoIj = SO3View::EvaluateRTp(tj_corrected_ns,
                                         spline_meta_.segments.at(seg_idx[1]),
                                         parameters + R_offset[1], nullptr);
          p_IjinG = R3View::evaluate(tj_corrected_ns,
                                     spline_meta_.segments.at(seg_idx[1]),
                                     parameters + P_offset[1], nullptr);
        }

        Vec3d gyro_i, gyro_j;
        Vec3d vel_i, vel_j;
        if (jacobians && jacobians[Knot_offset + 1])
        {
          gyro_i = SO3View::VelocityBody(ti_corrected_ns,
                                         spline_meta_.segments.at(seg_idx[0]),
                                         parameters + R_offset[0]);
          vel_i = R3View::velocity(ti_corrected_ns,
                                   spline_meta_.segments.at(seg_idx[0]),
                                   parameters + P_offset[0]);

          gyro_j = SO3View::VelocityBody(tj_corrected_ns,
                                         spline_meta_.segments.at(seg_idx[1]),
                                         parameters + R_offset[1]);
          vel_j = R3View::velocity(tj_corrected_ns,
                                   spline_meta_.segments.at(seg_idx[1]),
                                   parameters + P_offset[1]);
        }

        SO3d S_ItoC = S_CtoI.inverse();
        SO3d S_GtoCj = S_ItoC * S_GtoIj;
        Vec3d x_j = S_GtoCj * (p_G - p_IjinG) - S_ItoC * p_CinI;
        // Vec3d p_M =
        //     S_CtoI.inverse() * ((S_GtoIj * (p_G - p_IjinG)) - p_CinI);

        Eigen::Map<Eigen::Vector2d> residual(residuals);
        double depth_j_inv = 1.0 / x_j.z();
        residual = (x_j * depth_j_inv).head<2>() - p_j_.head<2>();

        if (jacobians)
        {
          for (size_t i = 0; i < kont_num; ++i)
          {
            if (jacobians[i])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i]);
              jac_kont_R.setZero();
            }
            if (jacobians[i + kont_num])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + kont_num]);
              jac_kont_p.setZero();
            }
          }
        }

        if (jacobians)
        {
          Eigen::Matrix<double, 2, 3> J_v;
          J_v.block<2, 2>(0, 0) = depth_j_inv * Eigen::Matrix2d::Identity();
          J_v.block<2, 1>(0, 2) = -depth_j_inv * depth_j_inv * x_j.head<2>();

          Eigen::Matrix<double, 2, 3> jac_lhs_R[2];
          Eigen::Matrix<double, 2, 3> jac_lhs_P[2];

          // 
          jac_lhs_R[0] = -J_v * (S_GtoCj * S_IitoG).matrix() * SO3::hat(p_Ii);
          jac_lhs_P[0] = J_v * S_GtoCj.matrix();

          // 
          jac_lhs_R[1] = J_v * S_GtoCj.matrix() * SO3::hat(p_G - p_IjinG);
          jac_lhs_P[1] = -J_v * S_GtoCj.matrix();

          ///[step1] jacobians of control points
          for (int seg = 0; seg < 2; ++seg)
          {
            /// Rotation control point
            size_t pre_idx_R = R_offset[seg] + J_R[seg].start_idx;
            for (size_t i = 0; i < SplineOrder; i++)
            {
              size_t idx = pre_idx_R + i;
              if (jacobians[idx])
              {
                Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                    jacobians[idx]);
                Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_temp;
                /// 2*3 3*3
                J_temp = jac_lhs_R[seg] * J_R[seg].d_val_d_knot[i];
                J_temp = (sqrt_info * J_temp).eval();

                jac_kont_R.block<2, 3>(0, 0) += J_temp;
              }
            }

            /// position control point
            size_t pre_idx_P = P_offset[seg] + J_p[seg].start_idx;
            for (size_t i = 0; i < SplineOrder; i++)
            {
              size_t idx = pre_idx_P + i;
              if (jacobians[idx])
              {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                    jacobians[idx]);

                Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_temp;
                /// 1*1 2*3
                J_temp = J_p[seg].d_val_d_knot[i] * jac_lhs_P[seg];
                J_temp = (sqrt_info * J_temp).eval();

                jac_kont_p += J_temp;
              }
            }
          }

          ///[step2] jacobians of inverse depth
          if (jacobians[Knot_offset])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 1>> jac_depth_inv(
                jacobians[Knot_offset]);
            jac_depth_inv.setZero();

            Vec3d J_Xm_d = -(S_GtoCj * S_IitoG * S_CtoI).matrix() * x_ci / d_inv;
            /// 2*3 3*1
            jac_depth_inv.block<2, 1>(0, 0) = J_v * J_Xm_d;
            jac_depth_inv = (sqrt_info * jac_depth_inv).eval();
          }

          // [step3] time offset
          if (jacobians[Knot_offset + 1])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 1>> jac_t_offset(
                jacobians[Knot_offset + 1]);
            jac_t_offset.setZero();

            Mat3d Ri_dot = S_IitoG.matrix() * SO3d::hat(gyro_i);
            Mat3d Rj_dot = S_GtoIj.inverse().matrix() * SO3d::hat(gyro_j);
            // t_j
            Vec3d J_tj = S_ItoC.matrix() * Rj_dot.transpose() * (p_G - p_IjinG) -
                         S_GtoCj * vel_j;
            // t_i
            Vec3d J_ti = S_GtoCj.matrix() * (Ri_dot * p_Ii + vel_i);
            jac_t_offset = 1e-9 * sqrt_info * J_v * (J_ti + J_tj);
          }
        }

        residual = (sqrt_info * residual).eval();
        return true;
      }

      static void SetParam(SO3d _S_CtoI, Vec3d _p_CinI)
      {
        init_flag = true;
        S_CtoI = _S_CtoI;
        p_CinI = _p_CinI;
      }

      // double focal_length = 450.;
      static inline Eigen::Matrix2d sqrt_info =
          450. / 1.5 * Eigen::Matrix2d::Identity();

    private:
      static inline bool init_flag = false;

      static inline SO3d S_CtoI;
      static inline Vec3d p_CinI;

      int64_t t_i_ns_;
      Eigen::Vector3d p_i_;
      int64_t t_j_ns_;
      Eigen::Vector3d p_j_;

      SplineMeta<SplineOrder> spline_meta_;
    };

    class Image3D2DFactor : public ceres::CostFunction,
                            So3SplineView,
                            RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;

      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using Mat3d = Eigen::Matrix<double, 3, 3>;
      using SO3d = Sophus::SO3<double>;

      Image3D2DFactor(const int64_t t_j_ns, const Eigen::Vector3d &p_j,
                      const SplineMeta<SplineOrder> &spline_meta)
          : t_j_ns_(t_j_ns), p_j_(p_j), spline_meta_(spline_meta)
      {
        assert(init_flag && "Image3D2DFactor not init param");

        /// 
        set_num_residuals(2);

        /// 
        size_t kont_num = spline_meta.NumParameters();
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
        mutable_parameter_block_sizes()->push_back(3); // p_inG
        mutable_parameter_block_sizes()->push_back(1); // time offset
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        size_t Knot_offset = 2 * spline_meta_.NumParameters();
        Eigen::Map<const Vec3d> p_G(parameters[Knot_offset]);
        double time_offset_in_ns = parameters[Knot_offset + 1][0];

        int64_t tj_corrected_ns = t_j_ns_ + (int64_t)time_offset_in_ns;

        size_t kont_num = spline_meta_.NumParameters();

        /// 
        SO3d S_GtoIj;
        Vec3d p_IjinG = Vec3d::Zero();
        if (jacobians)
        {
          // rhs = p_G - p_IjinG
          S_GtoIj = SO3View::EvaluateRTp(
              tj_corrected_ns, spline_meta_.segments.at(0), parameters, &J_R);
          p_IjinG = R3View::evaluate(tj_corrected_ns, spline_meta_.segments.at(0),
                                     parameters + kont_num, &J_p);
        }
        else
        {
          S_GtoIj = SO3View::EvaluateRTp(
              tj_corrected_ns, spline_meta_.segments.at(0), parameters, nullptr);
          p_IjinG = R3View::evaluate(tj_corrected_ns, spline_meta_.segments.at(0),
                                     parameters + kont_num, nullptr);
        }

        Vec3d gyro_j, vel_j;
        if (jacobians && jacobians[Knot_offset + 1])
        {
          gyro_j = SO3View::VelocityBody(tj_corrected_ns,
                                         spline_meta_.segments.at(0), parameters);
          vel_j = R3View::velocity(tj_corrected_ns, spline_meta_.segments.at(0),
                                   parameters + kont_num);
        }

        SO3d S_ItoC = S_CtoI.inverse();
        SO3d S_GtoCj = S_ItoC * S_GtoIj;
        Vec3d x_j = S_GtoCj * (p_G - p_IjinG) - S_ItoC * p_CinI;

        Eigen::Map<Eigen::Vector2d> residual(residuals);
        double depth_j_inv = 1.0 / x_j.z();
        residual = (x_j * depth_j_inv).head<2>() - p_j_.head<2>();

        if (jacobians)
        {
          for (size_t i = 0; i < kont_num; ++i)
          {
            if (jacobians[i])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i]);
              jac_kont_R.setZero();
            }
            if (jacobians[i + kont_num])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + kont_num]);
              jac_kont_p.setZero();
            }
          }
        }

        if (jacobians)
        {
          Eigen::Matrix<double, 2, 3> J_v;
          J_v.block<2, 2>(0, 0) = depth_j_inv * Eigen::Matrix2d::Identity();
          J_v.block<2, 1>(0, 2) = -depth_j_inv * depth_j_inv * x_j.head<2>();

          Eigen::Matrix<double, 2, 3> jac_lhs_R;
          Eigen::Matrix<double, 2, 3> jac_lhs_P;

          // 
          jac_lhs_R = J_v * S_GtoCj.matrix() * SO3::hat(p_G - p_IjinG);
          jac_lhs_P = -J_v * S_GtoCj.matrix();

          /// Rotation control point
          for (size_t i = 0; i < SplineOrder; i++)
          {
            size_t idx = J_R.start_idx + i;
            if (jacobians[idx])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[idx]);
              Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_temp;
              /// 2*3 3*3
              J_temp = jac_lhs_R * J_R.d_val_d_knot[i];
              J_temp = (sqrt_info * J_temp).eval();

              jac_kont_R.block<2, 3>(0, 0) += J_temp;
            }
          }

          /// position control point
          for (size_t i = 0; i < SplineOrder; i++)
          {
            size_t idx = J_p.start_idx + kont_num + i;
            if (jacobians[idx])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[idx]);

              Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_temp;
              /// 1*1 2*3
              J_temp = J_p.d_val_d_knot[i] * jac_lhs_P;
              J_temp = (sqrt_info * J_temp).eval();

              jac_kont_p += J_temp;
            }
          }

          /// jacobian of p_G
          if (jacobians[Knot_offset])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_p_G(
                jacobians[Knot_offset]);
            jac_p_G.setZero();

            /// 2*3 3*3
            jac_p_G = J_v * S_GtoCj.matrix();
            jac_p_G = (sqrt_info * jac_p_G).eval();
          }

          // jacobian of time offset
          if (jacobians[Knot_offset + 1])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 1>> jac_t_offset(
                jacobians[Knot_offset + 1]);
            jac_t_offset.setZero();

            Mat3d Rj_dot = S_GtoIj.inverse().matrix() * SO3d::hat(gyro_j);
            // t_j
            Vec3d J_tj = S_ItoC.matrix() * Rj_dot.transpose() * (p_G - p_IjinG) -
                         S_GtoCj * vel_j;
            jac_t_offset = 1e-9 * sqrt_info * J_v * J_tj;
          }
        }

        residual = (sqrt_info * residual).eval();
        return true;
      }

      static void SetParam(SO3d _S_CtoI, Vec3d _p_CinI)
      {
        init_flag = true;
        S_CtoI = _S_CtoI;
        p_CinI = _p_CinI;
      }

      // double focal_length = 450.;
      static inline Eigen::Matrix2d sqrt_info =
          450. / 1.5 * Eigen::Matrix2d::Identity();

    private:
      static inline bool init_flag = false;

      static inline SO3d S_CtoI;
      static inline Vec3d p_CinI;

      int64_t t_j_ns_;
      Eigen::Vector3d p_j_;

      SplineMeta<SplineOrder> spline_meta_;
    };

    class ImageFeatureOnePoseFactor : public ceres::CostFunction,
                                      So3SplineView,
                                      RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;

      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using SO3d = Sophus::SO3<double>;

      ImageFeatureOnePoseFactor(const Eigen::Vector3d &p_i, const SO3d &S_IitoG,
                                const Eigen::Vector3d &p_IiinG,
                                const int64_t t_j_ns, const Eigen::Vector3d &p_j,
                                const SplineMeta<SplineOrder> &spline_meta)
          : p_i_(p_i),
            S_IitoG_(S_IitoG),
            p_IiinG_(p_IiinG),
            t_j_ns_(t_j_ns),
            p_j_(p_j),
            spline_meta_(spline_meta)
      {
        assert(init_flag && "ImageFeatureOnePoseFactor not init param");

        /// 
        set_num_residuals(2);

        /// 
        size_t kont_num = spline_meta.NumParameters();
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
        mutable_parameter_block_sizes()->push_back(1); // inverse depth
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        size_t Knot_offset = 2 * spline_meta_.NumParameters();
        double d_inv = parameters[Knot_offset][0];

        size_t kont_num = spline_meta_.NumParameters();

        size_t R_offset, P_offset;

        double u;
        spline_meta_.ComputeSplineIndex(t_j_ns_, R_offset, u);
        P_offset = R_offset + kont_num;

        Vec3d x_ci = p_i_ / d_inv;
        /// 
        Vec3d p_Ii = S_CtoI * x_ci + p_CinI;

        /// 
        Vec3d p_G = S_IitoG_ * p_Ii + p_IiinG_;
        SO3d S_GtoIj;
        Vec3d p_IjinG = Vec3d::Zero();
        if (jacobians)
        {
          // rhs = p_G - p_IjinG
          S_GtoIj = SO3View::EvaluateRTp(t_j_ns_, spline_meta_.segments.at(0),
                                         parameters + R_offset, &J_R);
          p_IjinG = R3View::evaluate(t_j_ns_, spline_meta_.segments.at(0),
                                     parameters + P_offset, &J_p);
        }
        else
        {
          S_GtoIj = SO3View::EvaluateRTp(t_j_ns_, spline_meta_.segments.at(0),
                                         parameters + R_offset, nullptr);
          p_IjinG = R3View::evaluate(t_j_ns_, spline_meta_.segments.at(0),
                                     parameters + P_offset, nullptr);
        }
        SO3d S_ItoC = S_CtoI.inverse();
        SO3d S_GtoCj = S_ItoC * S_GtoIj;
        Vec3d x_j = S_GtoCj * (p_G - p_IjinG) - S_ItoC * p_CinI;

        Eigen::Map<Eigen::Vector2d> residual(residuals);
        double depth_j_inv = 1.0 / x_j.z();
        residual = (x_j * depth_j_inv).head<2>() - p_j_.head<2>();

        if (jacobians)
        {
          for (size_t i = 0; i < kont_num; ++i)
          {
            if (jacobians[i])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i]);
              jac_kont_R.setZero();
            }
            if (jacobians[i + kont_num])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + kont_num]);
              jac_kont_p.setZero();
            }
          }
        }

        if (jacobians)
        {
          Eigen::Matrix<double, 2, 3> J_v;
          J_v.block<2, 2>(0, 0) = depth_j_inv * Eigen::Matrix2d::Identity();
          J_v.block<2, 1>(0, 2) = -depth_j_inv * depth_j_inv * x_j.head<2>();

          Eigen::Matrix<double, 2, 3> jac_lhs_R, jac_lhs_P;
          // 
          jac_lhs_R = J_v * S_GtoCj.matrix() * SO3::hat(p_G - p_IjinG);
          jac_lhs_P = -J_v * S_GtoCj.matrix();

          ///[step1] jacobians of control points

          /// Rotation control point
          size_t pre_idx_R = R_offset + J_R.start_idx;
          for (size_t i = 0; i < SplineOrder; i++)
          {
            size_t idx = pre_idx_R + i;
            if (jacobians[idx])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[idx]);
              Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_temp;
              /// 2*3 3*3
              J_temp = jac_lhs_R * J_R.d_val_d_knot[i];
              J_temp = (sqrt_info * J_temp).eval();

              jac_kont_R.block<2, 3>(0, 0) += J_temp;
            }
          }

          /// position control point
          size_t pre_idx_P = P_offset + J_p.start_idx;
          for (size_t i = 0; i < SplineOrder; i++)
          {
            size_t idx = pre_idx_P + i;
            if (jacobians[idx])
            {
              Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[idx]);

              Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_temp;
              /// 1*1 2*3
              J_temp = J_p.d_val_d_knot[i] * jac_lhs_P;
              J_temp = (sqrt_info * J_temp).eval();

              jac_kont_p += J_temp;
            }
          }

          ///[step2] jacobians of inverse depth
          if (jacobians[Knot_offset])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 1>> jac_depth_inv(
                jacobians[Knot_offset]);
            jac_depth_inv.setZero();

            Vec3d J_Xm_d = -(S_GtoCj * S_IitoG_ * S_CtoI).matrix() * x_ci / d_inv;
            /// 2*3 3*1
            jac_depth_inv.block<2, 1>(0, 0) = J_v * J_Xm_d;
            jac_depth_inv = (sqrt_info * jac_depth_inv).eval();
          }
        }

        residual = (sqrt_info * residual).eval();
        return true;
      }

      static void SetParam(SO3d _S_CtoI, Vec3d _p_CinI)
      {
        init_flag = true;
        S_CtoI = _S_CtoI;
        p_CinI = _p_CinI;
      }

      // double focal_length = 450.;
      static inline Eigen::Matrix2d sqrt_info =
          450. / 1.5 * Eigen::Matrix2d::Identity();

    private:
      static inline bool init_flag = false;

      static inline SO3d S_CtoI;
      static inline Vec3d p_CinI;

      Eigen::Vector3d p_i_;
      SO3d S_IitoG_;
      Eigen::Vector3d p_IiinG_;

      int64_t t_j_ns_;
      Eigen::Vector3d p_j_;

      SplineMeta<SplineOrder> spline_meta_;
    };

    class ImageDepthFactor : public ceres::SizedCostFunction<2, 1>
    {
    public:
      using Vec3d = Eigen::Matrix<double, 3, 1>;

      ImageDepthFactor(const Eigen::Vector3d &p_i, const Eigen::Vector3d &p_j,
                       const SO3d &S_CitoCj, const Eigen::Vector3d &p_CiinCj)
          : p_i_(p_i), p_j_(p_j), S_CitoCj_(S_CitoCj), p_CiinCj_(p_CiinCj) {}

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        double d_inv = parameters[0][0];

        Vec3d x_ci = p_i_ / d_inv;
        Vec3d x_j = S_CitoCj_ * x_ci + p_CiinCj_;
        double depth_j_inv = 1.0 / x_j.z();

        Eigen::Map<Eigen::Vector2d> residual(residuals);
        residual = (x_j * depth_j_inv).head<2>() - p_j_.head<2>();

        if (jacobians)
        {
          Eigen::Matrix<double, 2, 3> J_v;
          J_v.block<2, 2>(0, 0) = depth_j_inv * Eigen::Matrix2d::Identity();
          J_v.block<2, 1>(0, 2) = -depth_j_inv * depth_j_inv * x_j.head<2>();

          /// jacobians of inverse depth
          if (jacobians[0])
          {
            Eigen::Map<Eigen::Matrix<double, 2, 1>> jac_depth_inv(jacobians[0]);
            jac_depth_inv.setZero();

            Vec3d J_Xm_d = -S_CitoCj_.matrix() * x_ci / d_inv;
            /// 2*3 3*1
            jac_depth_inv.block<2, 1>(0, 0) = J_v * J_Xm_d;
            jac_depth_inv = (sqrt_info * jac_depth_inv).eval();
          }
        }

        residual = (sqrt_info * residual).eval();
        return true;
      }

      // double focal_length = 450.;
      static inline Eigen::Matrix2d sqrt_info =
          450. / 1.5 * Eigen::Matrix2d::Identity();

    private:
      Eigen::Vector3d p_i_;
      Eigen::Vector3d p_j_;
      SO3d S_CitoCj_;
      Eigen::Vector3d p_CiinCj_;
    };

    class EpipolarFactor : public ceres::CostFunction, So3SplineView, RdSplineView
    {
    public:
      using SO3View = So3SplineView;
      using R3View = RdSplineView;

      using Vec3d = Eigen::Matrix<double, 3, 1>;
      using Mat3d = Eigen::Matrix<double, 3, 3>;
      using SO3d = Sophus::SO3<double>;

      EpipolarFactor(const int64_t t_i_ns, const Eigen::Vector3d &x_i,
                     const Eigen::Vector3d &x_k, const SO3d &S_GtoCk,
                     const Eigen::Vector3d &p_CkinG,
                     const SplineMeta<SplineOrder> &spline_meta, double weight)
          : t_i_ns_(t_i_ns),
            x_i_(x_i),
            x_k_(x_k),
            S_GtoCk_(S_GtoCk),
            p_CkinG_(p_CkinG),
            spline_meta_(spline_meta),
            weight_(weight)
      {
        assert(init_flag && "EpipolarFactor not init param");
        /// 
        set_num_residuals(1);

        /// 
        size_t kont_num = spline_meta.NumParameters();
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < kont_num; ++i)
        {
          mutable_parameter_block_sizes()->push_back(3);
        }
      }

      virtual bool Evaluate(double const *const *parameters, double *residuals,
                            double **jacobians) const
      {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        size_t kont_num = spline_meta_.NumParameters();

        SO3d S_IitoG;
        Vec3d p_IiinG = Vec3d::Zero();
        if (jacobians)
        {
          S_IitoG = SO3View::EvaluateRp(t_i_ns_, spline_meta_.segments[0],
                                        parameters, &J_R);
          p_IiinG = R3View::evaluate(t_i_ns_, spline_meta_.segments[0],
                                     parameters + kont_num, &J_p);
        }
        else
        {
          S_IitoG = SO3View::EvaluateRp(t_i_ns_, spline_meta_.segments[0],
                                        parameters, nullptr);
          p_IiinG = R3View::evaluate(t_i_ns_, spline_meta_.segments[0],
                                     parameters + kont_num, nullptr);
        }

        // 
        Vec3d Rxi = S_GtoCk_ * S_IitoG * S_CtoI * x_i_;
        Eigen::Matrix3d t_hat =
            SO3::hat(S_GtoCk_ * (S_IitoG * p_CinI + p_IiinG - p_CkinG_));
        residuals[0] = x_k_.transpose() * t_hat * Rxi;

        residuals[0] *= weight_;

        if (jacobians)
        {
          for (size_t i = 0; i < kont_num; ++i)
          {
            if (jacobians[i])
            {
              Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[i]);
              jac_kont_R.setZero();
            }
            if (jacobians[i + kont_num])
            {
              Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[i + kont_num]);
              jac_kont_p.setZero();
            }
          }

          Vec3d jac_lhs = x_k_.transpose() * SO3::hat(Rxi) * S_GtoCk_.matrix();
          Vec3d jac_lhs_P = -jac_lhs;
          Vec3d jac_lhs_R = -x_k_.transpose() * t_hat *
                            (S_GtoCk_ * S_IitoG).matrix() * SO3::hat(S_CtoI * x_i_);
          jac_lhs_R += jac_lhs.transpose() * S_IitoG.matrix() * SO3::hat(p_CinI);

          /// Rotation control point
          for (size_t i = 0; i < kont_num; i++)
          {
            size_t idx = i;
            if (jacobians[idx])
            {
              Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
                  jacobians[idx]);
              jac_kont_R.setZero();

              /// 1*3 3*3
              jac_kont_R.block<1, 3>(0, 0) =
                  jac_lhs_R.transpose() * J_R.d_val_d_knot[i];
              jac_kont_R = (weight_ * jac_kont_R).eval();
            }
          }

          /// position control point
          for (size_t i = 0; i < kont_num; i++)
          {
            size_t idx = kont_num + i;
            if (jacobians[idx])
            {
              Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
                  jacobians[idx]);
              jac_kont_p.setZero();

              /// 1*1 1*3
              jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
              jac_kont_p = (weight_ * jac_kont_p).eval();
            }
          }
        }

        return true;
      }

      static void SetParam(SO3d _S_CtoI, Vec3d _p_CinI)
      {
        init_flag = true;
        S_CtoI = _S_CtoI;
        p_CinI = _p_CinI;
      }

    private:
      static inline bool init_flag = false;
      static inline SO3d S_CtoI;
      static inline Vec3d p_CinI;

      int64_t t_i_ns_;
      Eigen::Vector3d x_i_;

      Eigen::Vector3d x_k_;
      SO3d S_GtoCk_;
      Eigen::Vector3d p_CkinG_;

      SplineMeta<SplineOrder> spline_meta_;
      double weight_;
    };

  } // namespace analytic_derivative

} // namespace cocolic
