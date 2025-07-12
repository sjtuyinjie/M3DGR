/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

#include "precomp.h"
#include <opencv2/opencv.hpp>

  /** Just compute the norm of a vector
   * @param vec a vector of size 3 and any type T
   * @return
   */
  template<typename T>
  T
  inline
  norm_vec(const cv::Vec<T, 3> &vec)
  {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
  }

  /** Modify normals to make sure they point towards the camera
   * @param normals
   */
  template<typename T>
  inline
  void
  signNormal(const cv::Vec<T, 3> & normal_in, cv::Vec<T, 3> & normal_out)
  {
    cv::Vec<T, 3> res;
    if (normal_in[2] > 0)
      res = -normal_in / norm_vec(normal_in);
    else
      res = normal_in / norm_vec(normal_in);

    normal_out[0] = res[0];
    normal_out[1] = res[1];
    normal_out[2] = res[2];
  }

    /** Normalized normals
    * @param normals
    */
    template<typename T>
    inline
    void
    normalizedNormal(const cv::Vec<T, 3> & normal_in, cv::Vec<T, 3> & normal_out)
    {
        normal_out = normal_in / norm_vec(normal_in);
    }

  /** Given 3d points, compute their distance to the origin
   * @param points
   * @return
   */
  template<typename T>
  cv::Mat_<T>
  computeRadius(const cv::Mat &points)
  {
    typedef cv::Vec<T, 3> PointT;

    // Compute the
      cv::Size size(points.cols, points.rows);
      cv::Mat_<T> r(size);
    if (points.isContinuous())
      size = cv::Size(points.cols * points.rows, 1);
    for (int y = 0; y < size.height; ++y)
    {
      const PointT* point = points.ptr < PointT > (y), *point_end = points.ptr < PointT > (y) + size.width;
      T * row = r[y];
      for (; point != point_end; ++point, ++row)
        *row = norm_vec(*point);
    }

    return r;
  }

  // Compute theta and phi according to equation 3 of
  // ``Fast and Accurate Computation of Surface Normals from Range Images``
  // by H. Badino, D. Huber, Y. Park and T. Kanade
  template<typename T>
  void
  computeThetaPhi(int rows, int cols, const cv::Matx<T, 3, 3>& K, cv::Mat &cos_theta, cv::Mat &sin_theta,
                  cv::Mat &cos_phi, cv::Mat &sin_phi)
  {
    // Create some bogus coordinates
      cv::Mat depth_image = K(0, 0) * cv::Mat_ < T > ::ones(rows, cols); //fx
      cv::Mat points3d;
    cv::rgbd::depthTo3d(depth_image, cv::Mat(K), points3d);

    typedef cv::Vec<T, 3> Vec3T;


    cos_theta = cv::Mat_ < T > (rows, cols);
    sin_theta = cv::Mat_ < T > (rows, cols);
    cos_phi = cv::Mat_ < T > (rows, cols);
    sin_phi = cv::Mat_ < T > (rows, cols);
      cv::Mat r = computeRadius<T>(points3d);

    for (int y = 0; y < rows; ++y)
    {

      T *row_cos_theta = cos_theta.ptr < T > (y), *row_sin_theta = sin_theta.ptr < T > (y);
      T *row_cos_phi = cos_phi.ptr < T > (y), *row_sin_phi = sin_phi.ptr < T > (y);

      const Vec3T * row_points = points3d.ptr < Vec3T > (y), *row_points_end = points3d.ptr < Vec3T
          > (y) + points3d.cols;
      const T * row_r = r.ptr < T > (y);
      for (; row_points < row_points_end;
          ++row_cos_theta, ++row_sin_theta, ++row_cos_phi, ++row_sin_phi, ++row_points, ++row_r)
      {
        // In the paper, z goes away from the camera, y goes down, x goes right
        // OpenCV has the same conventions
        // Theta goes from z to x (and actually goes from -pi/2 to pi/2, phi goes from z to y
        float theta = (float)std::atan2(row_points->val[0], row_points->val[2]); //theta，x/z
        *row_cos_theta = std::cos(theta);
        *row_sin_theta = std::sin(theta);
        float phi = (float)std::asin(row_points->val[1] / (*row_r)); //phi， y/r
        *row_cos_phi = std::cos(phi);
        *row_sin_phi = std::sin(phi);
      }
    }
  }

//template<typename T>
class LIDAR_FALS
{
public:
    typedef cv::Matx<float, 3, 3> Mat33T;
    typedef cv::Vec<float, 9> Vec9T;
    typedef cv::Vec<float, 3> Vec3T;

    //(depth是数据类型
    LIDAR_FALS(int rows, int cols, int window_size, int depth, const cv::Mat &K)
    {}

    LIDAR_FALS(int rows, int cols, int window_size)
    : rows_(rows)
    , cols_(cols)
    , window_size_(window_size)
    {}

    LIDAR_FALS() {}

    ~LIDAR_FALS() {}

    ///计算M逆
    /** Compute cached data
     */
    virtual void
    cache()
    {
        std::cout << "build look up table.\n";

        // Compute theta and phi according to equation 3
        cv::Mat cos_theta, sin_theta, cos_phi, sin_phi;
        computeThetaPhi<float>(rows_, cols_, K_, cos_theta, sin_theta, cos_phi, sin_phi);

        computeMInverse(cos_theta, sin_theta, cos_phi, sin_phi);
    }

    void computeMInverse(cv::Mat &cos_theta, cv::Mat &sin_theta, cv::Mat &cos_phi, cv::Mat &sin_phi)
    {
        // Compute all the v_i for every points
        std::vector<cv::Mat> channels(3);
        channels[0] = cos_phi.mul(cos_theta);
        channels[1] = -cos_phi.mul(sin_theta);
        channels[2] = sin_phi;
        merge(channels, V_);

        // Compute M
        cv::Mat_<Vec9T> M(rows_, cols_);
        Mat33T VVt;
        const Vec3T * vec = V_[0];
        Vec9T * M_ptr = M[0], *M_ptr_end = M_ptr + rows_ * cols_;
        for (; M_ptr != M_ptr_end; ++vec, ++M_ptr)
        {
            VVt = (*vec) * vec->t(); //v * v_t
            *M_ptr = Vec9T(VVt.val);
        }

        ///todo BorderTypes::BORDER_TRANSPARENT, error
        int border_type = cv::BorderTypes::BORDER_TRANSPARENT;
        boxFilter(M, M, M.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

        // Compute M's inverse
        Mat33T M_inv;
        M_inv_.create(rows_, cols_);
        Vec9T * M_inv_ptr = M_inv_[0];
        for (M_ptr = &M(0); M_ptr != M_ptr_end; ++M_inv_ptr, ++M_ptr)
        {
            // We have a semi-definite matrix
            invert(Mat33T(M_ptr->val), M_inv, cv::DECOMP_CHOLESKY);
            *M_inv_ptr = Vec9T(M_inv.val);
        }
    }

    void saveMInverse(std::string dir, std::string filename)
    {
        std::vector<cv::Mat> mats(9);
        for (int i = 0; i < 9; ++i) {
            mats[i].create(M_inv_.rows, M_inv_.cols, CV_32F);
        }
        cv::Mat mat_vv(M_inv_.rows, M_inv_.cols, CV_32FC3);
        for (int i = 0; i < M_inv_.rows; ++i) {
            for (int j = 0; j < M_inv_.cols; ++j) {
                const Vec9T& tmp(M_inv_.at<Vec9T>(i, j));
                for (int k = 0; k < 9; ++k) {
                    mats[k].at<float>(i, j) = tmp(k);
                }
                mat_vv.at<Vec3T>(i, j) = V_.at<Vec3T>(i, j);
            }
        }

        //save M inverse
        for (int i = 0; i < 9; ++i) {
            std::string file_id(std::to_string(i));
            cv::FileStorage fs(dir + "/" + filename + "_M_" + file_id + ".xml", cv::FileStorage::WRITE);
            fs << filename + "_" + file_id << mats[i];
            fs.release();
        }
        //save v v_t
        cv::FileStorage fs(dir + "/" + filename + "_v" +  + ".xml", cv::FileStorage::WRITE);
        fs << filename + "_vv"  << mat_vv;
        fs.release();
    }

    bool loadMInverse(std::string dir, std::string filename)
    {
        std::vector<cv::Mat> mats(9);
        for (int i = 0; i < 9; ++i) {
            mats[i].create(rows_, cols_, CV_32F);
        }

        for (int i = 0; i < 9; ++i) {
            std::string file_id(std::to_string(i));
            cv::FileStorage fs(dir + "/" + filename + "_M_" + file_id + ".xml", cv::FileStorage::READ);
            if(!fs.isOpened())
            {
//                std::cerr << "ERROR: Wrong path to ring normal M file" << std::endl;
                return false;
            }
            fs[filename + "_" + file_id] >> mats[i];
            fs.release();
        }

        M_inv_.create(rows_, cols_);

        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_; ++j) {
                Vec9T& tmp(M_inv_.at<Vec9T>(i, j));
                for (int k = 0; k < 9; ++k) {
                    tmp(k) = mats[k].at<float>(i, j);
                }
            }
        }

        V_.create(rows_, cols_);
        cv::FileStorage fs(dir + "/" + filename + "_v" +  + ".xml", cv::FileStorage::READ);
        fs[filename + "_vv"] >> V_;
        fs.release();
        return true;
    }

    /** Compute the normals
     * @param r
     * @return
     */
    virtual void
    compute(const cv::Mat &r, cv::Mat & normals) const
    {
        // Compute B
        cv::Mat_<Vec3T> B(rows_, cols_);

        const float* row_r = r.ptr < float > (0), *row_r_end = row_r + rows_ * cols_;
        const Vec3T *row_V = V_[0];
        Vec3T *row_B = B[0];
        for (; row_r != row_r_end; ++row_r, ++row_B, ++row_V)
        {
            if (*row_r==FLT_MAX)
                *row_B = Vec3T();
            else
                *row_B = (*row_V) / (*row_r); //v_i / r_i
        }

        ///todo BorderTypes::BORDER_TRANSPARENT, error
        // Apply a box filter to B
        boxFilter(B, B, B.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

        // compute the Minv*B products
        row_r = r.ptr < float > (0);
        const Vec3T * B_vec = B[0];
        const Mat33T * M_inv = reinterpret_cast<const Mat33T *>(M_inv_.ptr(0));
        Vec3T *normal = normals.ptr<Vec3T>(0);
        for (; row_r != row_r_end; ++row_r, ++B_vec, ++normal, ++M_inv)
            if (*row_r==FLT_MAX)
            {
                (*normal)[0] = *row_r;
                (*normal)[1] = *row_r;
                (*normal)[2] = *row_r;
            }
            else
            {
                Mat33T Mr = *M_inv;
                Vec3T Br = *B_vec;
                Vec3T MBr(Mr(0, 0) * Br[0] + Mr(0, 1)*Br[1] + Mr(0, 2)*Br[2],
                          Mr(1, 0) * Br[0] + Mr(1, 1)*Br[1] + Mr(1, 2)*Br[2],
                          Mr(2, 0) * Br[0] + Mr(2, 1)*Br[1] + Mr(2, 2)*Br[2]);
                //actually, can not flip normal here, because the point position is unknown
                signNormal(MBr, *normal);
            }
    }

    /** Compute the normals
 * @param r
 * @return
 */
    virtual void
    compute(const cv::Mat &r, cv::Mat & normals, cv::Mat & residual) const
    {
        // Compute B
        cv::Mat_<Vec3T> B(rows_, cols_);

        const float* row_r = r.ptr < float > (0), *row_r_end = row_r + rows_ * cols_;
        const Vec3T *row_V = V_[0];
        Vec3T *row_B = B[0];
        for (; row_r != row_r_end; ++row_r, ++row_B, ++row_V)
        {
            if (*row_r==FLT_MAX)
                *row_B = Vec3T();
            else
                *row_B = (*row_V) / (*row_r); //v_i / r_i
        }

        ///todo BorderTypes::BORDER_TRANSPARENT, error
        // Apply a box filter to B
        boxFilter(B, B, B.depth(), cv::Size(window_size_, window_size_), cv::Point(-1, -1), false);

        // compute the Minv*B products
        row_r = r.ptr < float > (0);
        const Vec3T * B_vec = B[0];
        const Mat33T * M_inv = reinterpret_cast<const Mat33T *>(M_inv_.ptr(0));
        Vec3T *normal = normals.ptr<Vec3T>(0);
        for (; row_r != row_r_end; ++row_r, ++B_vec, ++normal, ++M_inv) {
            if (*row_r==FLT_MAX) {
                (*normal)[0] = *row_r;
                (*normal)[1] = *row_r;
                (*normal)[2] = *row_r;
            } else {
                Mat33T Mr = *M_inv;
                Vec3T Br = *B_vec;
                Vec3T MBr(Mr(0, 0) * Br[0] + Mr(0, 1) * Br[1] + Mr(0, 2) * Br[2],
                          Mr(1, 0) * Br[0] + Mr(1, 1) * Br[1] + Mr(1, 2) * Br[2],
                          Mr(2, 0) * Br[0] + Mr(2, 1) * Br[1] + Mr(2, 2) * Br[2]);
//                signNormal(MBr, *normal);
                normalizedNormal(MBr, *normal);
            }
        }

//        {
//            row_V = V_[0];
//            row_r = r.ptr < float > (0);
//            Vec3T *normal = normals.ptr<Vec3T>(0);
//            float *res = residual.ptr<float>(0);
//            for (; row_r != row_r_end; ++row_r, ++normal, ++row_V, ++res) {
//                float e = row_V->dot(*normal) - (1.0 / *row_r);
//                *res = e * e;
//            }
//        }
    }


    void setRowsCols(const int& rows, const int& cols) {
        rows_ = rows;
        cols_ = cols;
    }

private:
    int rows_, cols_;
    int depth_ = CV_32F;
    cv::Mat K_, K_ori_;
    int window_size_;
    cv::Mat_<Vec3T> V_; //sin(theta) * cos(phi), sin(phi), cos(theta) * cos(phi)
    cv::Mat_<Vec9T> M_inv_; //M^-1
};