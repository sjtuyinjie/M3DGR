/*
 * atan_camera.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: cforster
 */


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vikit/atan_camera.h>
#include <vikit/math_utils.h>

namespace vk {

ATANCamera::
ATANCamera(double width, double height,
           double fx, double fy,
           double cx, double cy,
           double s) :
           AbstractCamera(width, height),
           fx_(width*fx), fy_(height*fy),
           fx_inv_(1.0/fx_), fy_inv_(1.0/fy_),
           cx_(cx*width - 0.5), cy_(cy*height - 0.5),
           s_(s), s_inv_(1.0/s_)
{
  if(s_ != 0.0)
  {
    tans_ = 2.0 * tan(s_ / 2.0);
    tans_inv_ = 1.0 / tans_;
    s_inv_ = 1.0 / s_;
    distortion_ = true;
  }
  else
  {
    s_inv_ = 0.0;
    tans_ = 0.0;
    distortion_ = false;
  }
}

ATANCamera::
~ATANCamera()
{}

Vector3d ATANCamera::
cam2world(const double& x, const double& y) const
{
  Vector2d dist_cam((x - cx_) * fx_inv_,
                    (y - cy_) * fy_inv_);
  double dist_r = dist_cam.norm();
  double r = invrtrans(dist_r);
  double d_factor;
  if(dist_r > 0.01)
    d_factor =  r / dist_r;
  else
    d_factor = 1.0;
  return unproject2d(d_factor * dist_cam).normalized();
}

Vector3d ATANCamera::
cam2world (const Vector2d& px) const
{
  return cam2world(px[0], px[1]);
}

Vector2d ATANCamera::
world2cam(const Vector3d& xyz_c) const
{
  return world2cam(project2d(xyz_c));
}

Vector2d ATANCamera::
world2cam(const Vector2d& uv) const
{
  double r = uv.norm();
  double factor = rtrans_factor(r);
  Vector2d dist_cam = factor * uv;
  return Vector2d(cx_ + fx_ * dist_cam[0],
                  cy_ + fy_ * dist_cam[1]);
}

} /* end vk */
