/*
 * img_align.cpp
 *
 *  Created on: Aug 22, 2012
 *      Author: cforster
 */

#include <vector>
#include <string>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>
#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>
#include <vikit/img_align.h>
#include <sophus/se3.h>

namespace vk {

/*******************************************************************************
 * Forward Compositional
 */
ForwardCompositionalSE3::
ForwardCompositionalSE3( vector<PinholeCamera>& cam_pyr,
                         vector<cv::Mat>& depth_pyr,
                         vector<cv::Mat>& img_pyr,
                         vector<cv::Mat>& tpl_pyr,
                         vector<cv::Mat>& img_pyr_dx,
                         vector<cv::Mat>& img_pyr_dy,
                         SE3& init_model,
                         int n_levels,
                         int n_iter,
                         float res_thresh,
                         bool display,
                         Method method,
                         int test_id) :
      cam_pyr_(cam_pyr),
      depth_pyr_(depth_pyr),
      img_pyr_(img_pyr),
      tpl_pyr_(tpl_pyr),
      img_pyr_dx_(img_pyr_dx),
      img_pyr_dy_(img_pyr_dy),
      display_(display),
      log_(test_id < 0),
      res_thresh_(res_thresh)
{
  n_iter_ = n_iter;
  method_ = method;

  // Init Performance Monitor
#if 0
  if(log_)
  {
    permon_.init("forward", ros::package::getPath("rpl_examples") + "/trace/img_align/data",
                 test_id, true);
    permon_.addLog("iter");
    permon_.addLog("level");
    permon_.addLog("mu");
    permon_.addLog("chi2");
    permon_.addLog("trials");
  }
#endif

  runOptimization(init_model);

}

ForwardCompositionalSE3::
ForwardCompositionalSE3( vector<PinholeCamera>& cam_pyr,
                         vector<cv::Mat>& depth_pyr,
                         vector<cv::Mat>& img_pyr,
                         vector<cv::Mat>& tpl_pyr,
                         vector<cv::Mat>& img_pyr_dx,
                         vector<cv::Mat>& img_pyr_dy,
                         int n_levels,
                         int n_iter,
                         float res_thresh,
                         bool display,
                         Method method,
                         int test_id) :
      cam_pyr_(cam_pyr),
      depth_pyr_(depth_pyr),
      img_pyr_(img_pyr),
      tpl_pyr_(tpl_pyr),
      img_pyr_dx_(img_pyr_dx),
      img_pyr_dy_(img_pyr_dy),
      display_(display),
      log_(test_id < 0),
      res_thresh_(res_thresh)
{
  n_iter_ = n_iter;
  method_ = method;

  // Init Performance Monitor
#if 0
  if(log_)
  {
    permon_.init("forward", ros::package::getPath("rpl_examples") + "/trace/img_align/data",
                 test_id, true);
    permon_.addLog("iter");
    permon_.addLog("level");
    permon_.addLog("mu");
    permon_.addLog("chi2");
    permon_.addLog("trials");
  }
#endif

}

void ForwardCompositionalSE3::
runOptimization(SE3& model, int levelBegin, int levelEnd)
{
  if(levelBegin < 0 || levelBegin > n_levels_-1)
    levelBegin = n_levels_-1;
  if(levelEnd < 0)
    levelEnd = 1;

  // Perform Pyramidal optimization
  for(level_=levelBegin; level_>=levelEnd; --level_)
  {
    mu_ = 0.1;
    cout << endl << "PYRAMID LEVEL " << level_
         << endl << "---------------" << endl;
    optimize(model);
  }
}

double ForwardCompositionalSE3::
computeResiduals (const SE3& model, bool linearize_system, bool compute_weight_scale)
{
  // Warp the image such that it aligns with the template image
  double chi2 = 0;
  size_t n_pixels = 0;

  if(linearize_system)
    resimg_ = cv::Mat(tpl_pyr_[level_].size(), CV_32F, cv::Scalar(1));

  for( int v=0; v<depth_pyr_[level_].rows; ++v )
  {
    for( int u=0; u<depth_pyr_[level_].cols; ++u )
    {
      // compute pixel location in new img
      cv::Vec3f cv_float3 = depth_pyr_[level_].at<cv::Vec3f>(v,u);
      Vector3d xyz_tpl(cv_float3[0], cv_float3[1], cv_float3[2]);
      Vector3d xyz_img(model*xyz_tpl);
      Vector2f uv_img_pyr = cam_pyr_[level_].world2cam(xyz_img).cast<float>(); // apply cam model
      if( cam_pyr_[level_].isInFrame(uv_img_pyr.cast<int>(), 2) )
      {
        // compare image values
        float intensity_tpl =
            tpl_pyr_[level_].at<float>(v,u);
        float intensity_img =
            interpolateMat_32f(img_pyr_[level_], uv_img_pyr[0], uv_img_pyr[1]);

        // compute residual (opposite to 2d case because of other jacobian)
        float res = intensity_tpl-intensity_img;

        // robustification
        if(res > res_thresh_)  res = res_thresh_;
        if(res < -res_thresh_) res = -res_thresh_;
        chi2 += res*res;
        n_pixels++;

        if(linearize_system)
        {
          // get gradient of warped image (~gradient at warped position)
          float dx = 0.5*interpolateMat_32f(img_pyr_dx_[level_], uv_img_pyr[0], uv_img_pyr[1]);
          float dy = 0.5*interpolateMat_32f(img_pyr_dy_[level_], uv_img_pyr[0], uv_img_pyr[1]);

          // evaluate jacobian
          Matrix<double,2,6> frame_jac;
          frameJac_xyz2uv(xyz_img, cam_pyr_[level_].fx(), frame_jac);

          // compute steppest descent images
          Vector6d J = dx*frame_jac.row(0) + dy*frame_jac.row(1);

          // compute Hessian and
          H_ += J*J.transpose();
          Jres_ += J*res;

          resimg_.at<float>(v,u) = -res;
        }
      }
    }
  }
  chi2 /= n_pixels;
  return chi2;
}

int ForwardCompositionalSE3::
solve()
{
  x_ = H_.ldlt().solve(-Jres_);
  if((bool) std::isnan((double) x_[0]))
    return 0;
  return 1;
}

void ForwardCompositionalSE3::
update(const ModelType& old_model,  ModelType& new_model)
{
  new_model = SE3::exp(x_)*(old_model);
}

void ForwardCompositionalSE3::
startIteration()
{
#if 0
  if(log_)
    permon_.newMeasurement();
#endif
}

void ForwardCompositionalSE3::
finishIteration()
{
#if 0
  if(log_)
  {
    permon_.log("iter", iter_);
    permon_.log("level", level_);
    permon_.log("mu", mu_);
    permon_.log("chi2", chi2_);
    permon_.log("trials", n_trials_);
  }
#endif

  if(display_)
  {
    cv::namedWindow("residuals", cv::WINDOW_AUTOSIZE);
    cv::imshow("residuals", resimg_*3);
    cv::waitKey(0);
  }
}

/*******************************************************************************
 * Efficient Second Order Minimization (ESM)
 */
SecondOrderMinimisationSE3::
SecondOrderMinimisationSE3( vector<PinholeCamera>& cam_pyr,
                            vector<cv::Mat>& depth_pyr,
                            vector<cv::Mat>& img_pyr,
                            vector<cv::Mat>& tpl_pyr,
                            vector<cv::Mat>& img_pyr_dx,
                            vector<cv::Mat>& img_pyr_dy,
                            vector<cv::Mat>& tpl_pyr_dx,
                            vector<cv::Mat>& tpl_pyr_dy,
                            SE3& init_model,
                            int n_levels,
                            int n_iter,
                            float res_thresh,
                            bool display,
                            Method method,
                            int test_id) :
      cam_pyr_(cam_pyr),
      depth_pyr_(depth_pyr),
      img_pyr_(img_pyr),
      tpl_pyr_(tpl_pyr),
      img_pyr_dx_(img_pyr_dx),
      img_pyr_dy_(img_pyr_dy),
      tpl_pyr_dx_(tpl_pyr_dx),
      tpl_pyr_dy_(tpl_pyr_dy),
      display_(display),
      log_(test_id < 0),
      res_thresh_(res_thresh)
{
  n_iter_ = n_iter;
  method_ = method;
  verbose_ = false;

#if 0
  if(log_)
  {
    // Init Performance Monitor
    permon_.init("esm", ros::package::getPath("rpl_examples") + "/trace/img_align/data",
                 test_id, true);
    permon_.addLog("iter");
    permon_.addLog("level");
    permon_.addLog("mu");
    permon_.addLog("chi2");
    permon_.addLog("trials");
  }
#endif

  // perform pyramidal optimization
  for(level_=n_levels-1; level_>2; --level_)
  //level_ = n_levels-1;
  {
    // Optimize
    mu_ = 0.01f;
    if(display_)
    {
      cout << endl << "PYRAMID LEVEL " << level_
           << endl << "patch-width = " << img_pyr_[level_].cols
           << endl << "---------------" << endl;
    }
    optimize(init_model);
  }
}

double SecondOrderMinimisationSE3::
computeResiduals (const SE3& model, bool linearize_system, bool compute_weight_scale)
{
  // Warp the image such that it aligns with the template image
  double chi2 = 0;
  size_t n_pixels = 0;

  // TODO: to improve access speed, use a pointer and increment every iteration

  // Compute Warp
  cv::Mat mask = cv::Mat_<bool>(tpl_pyr_[level_].rows, tpl_pyr_[level_].cols, false);
  cv::Mat img_warped = cv::Mat_<float>(tpl_pyr_[level_].rows, tpl_pyr_[level_].cols, 1.0);

  for( int v=0; v<depth_pyr_[level_].rows; ++v )
  {
    for( int u=0; u<depth_pyr_[level_].cols; ++u )
    {
      // compute pixel location in new img
      cv::Vec3f cv_float3 = depth_pyr_[level_].at<cv::Vec3f>(v,u);
      Vector3d xyz_tpl(cv_float3[0], cv_float3[1], cv_float3[2]);
      Vector3d xyz_img(model*xyz_tpl);
      Vector2f uv_img_pyr = cam_pyr_[level_].world2cam(xyz_img).cast<float>(); // apply cam model
      if( cam_pyr_[level_].isInFrame(uv_img_pyr.cast<int>(), 1) )
      {
        img_warped.at<float>(v,u) = interpolateMat_32f(img_pyr_[level_], uv_img_pyr[0], uv_img_pyr[1]);

        if( cam_pyr_[level_].isInFrame(uv_img_pyr.cast<int>(), 2) )
          mask.at<bool>(v,u) = true;
      }
    }
  }

  // Compute Warp derivative
  cv::Mat img_warped_dx, img_warped_dy;
  cv::Sobel(img_warped, img_warped_dx, CV_32F, 1, 0, 1);
  cv::Sobel(img_warped, img_warped_dy, CV_32F, 0, 1, 1);

  // Compute Jacobian
  if(linearize_system)
    resimg_ = cv::Mat_<float>(tpl_pyr_[level_].size(), 1.0);

  for( int v=0; v<depth_pyr_[level_].rows; ++v )
  {
    for( int u=0; u<depth_pyr_[level_].cols; ++u )
    {
      if (mask.at<bool>(v,u))
      {
        // compare image values
        float intensity_tpl = tpl_pyr_[level_].at<float>(v,u);
        float intensity_img = img_warped.at<float>(v,u);

        // compute residual  (opposite to 2d case because of other jacobian)
        float res = intensity_tpl-intensity_img;

        // robustification
        if(res > res_thresh_)  res = res_thresh_;
        if(res < -res_thresh_) res = -res_thresh_;
        chi2 += res*res;
        n_pixels++;

        if(linearize_system)
        {
          // 0.25 because we have two 0.5 factors. First from adding the two gradients
          // and the second when we compute the sobel mask
          float dx = 0.25*(tpl_pyr_dx_[level_].at<float>(v,u) + img_warped_dx.at<float>(v,u));
          float dy = 0.25*(tpl_pyr_dy_[level_].at<float>(v,u) + img_warped_dy.at<float>(v,u));

          // evaluate jacobian
          cv::Vec3f cv_float3 = depth_pyr_[level_].at<cv::Vec3f>(v,u);
          Vector3d xyz_tpl(cv_float3[0], cv_float3[1], cv_float3[2]);
          Vector3d xyz_img(model*xyz_tpl);
          Matrix<double,2,6> frame_jac;
          frameJac_xyz2uv(xyz_tpl, cam_pyr_[level_].fx(), frame_jac);

          // compute steppest descent images
          Vector6d J = dx*frame_jac.row(0) + dy*frame_jac.row(1);

          // compute Hessian
          H_ += J*J.transpose();
          Jres_ += J*res;
          resimg_.at<float>(v,u) = res;
        }
      }
    }
  }
  chi2 /= n_pixels;
  return chi2;
}

int SecondOrderMinimisationSE3::
solve()
{
  x_ = H_.ldlt().solve(-Jres_);
  if((bool) std::isnan((double) x_[0]))
    return 0;
  return 1;
}

void SecondOrderMinimisationSE3::
update(const ModelType& old_model,  ModelType& new_model)
{
  new_model = SE3::exp(x_)*old_model;
}

void SecondOrderMinimisationSE3::
startIteration()
{
#if 0
  if(log_)
    permon_.newMeasurement();
#endif
}

void SecondOrderMinimisationSE3::
finishIteration()
{
#if 0
  if(log_)
  {
    permon_.log("iter", iter_);
    permon_.log("level", level_);
    permon_.log("mu", mu_);
    permon_.log("chi2", chi2_);
    permon_.log("trials", n_trials_);
  }
#endif

  if(display_)
  {
    cv::namedWindow("residuals", cv::WINDOW_AUTOSIZE);
    cv::imshow("residuals", resimg_*3);
    cv::waitKey(0);
  }
}

} // end namespace vk
