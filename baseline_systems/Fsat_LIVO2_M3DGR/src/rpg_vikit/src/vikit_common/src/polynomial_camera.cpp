/*
 * polynomial_camera.cpp
 *
 *  Created on: January 26, 2023
 *      Author: xuankuzcr
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vikit/polynomial_camera.h>
#include <vikit/math_utils.h>

namespace vk {

// PolynomialCamera::
// PolynomialCamera(double width, double height, double scale, 
//               double fx, double fy,
//               double cx, double cy, double skew,
//               double k2, double k3, double k4, double k5, double k6, double k7) :
//               AbstractCamera(width * scale , height * scale, scale),
//               fx_(fx * scale), fy_(fy * scale), cx_(cx * scale), cy_(cy * scale), skew_(skew * scale),
//               distortion_(fabs(k2) > 0.0000001)
// {
//   cout << "scale: " << scale << endl;
//   k2_ = k2; k3_ = k3; k4_ = k4; k5_ = k5; k6_ = k6; k7_ = k7; 
// }

PolynomialCamera::
PolynomialCamera(double width, double height, // double scale, 
              double fx, double fy,
              double cx, double cy, double skew,
              double k2, double k3, double k4, double k5, double k6, double k7) :
              AbstractCamera(width, height, 1.0),
              fx_(fx), fy_(fy), cx_(cx), cy_(cy), skew_(skew),
              distortion_(fabs(k2) > 0.0000001)
{
  // cout << "scale: " << scale << endl;
  k2_ = k2; k3_ = k3; k4_ = k4; k5_ = k5; k6_ = k6; k7_ = k7; 
}

PolynomialCamera::
~PolynomialCamera()
{}

Vector3d PolynomialCamera::
cam2world(const double& u, const double& v) const
{
  Vector3d xyz;
  if(!distortion_)
  {
    // xyz[0] = (u - cx_)/fx_;
    // xyz[1] = (v - cy_)/fy_;
    // xyz[2] = 1.0;
    xyz[1] = (v - cy_)/fy_;
    xyz[0] = (u - cx_ - xyz[1]*skew_)/fx_;
    xyz[2] = 1.0;
  }
  else
  {
    double y = (v - cy_)/fy_;
    double x = (u - cx_ - y*skew_)/fx_;

    const double thetad = std::sqrt(x * x + y * y);
    double theta = thetad;
    for (int i = 0; i < 7; ++i)
    {
      const double theta2 = theta * theta;
      const double theta3 = theta2 * theta;
      const double theta4 = theta3 * theta;
      const double theta5 = theta4 * theta;
      const double theta6 = theta5 * theta;
      theta = thetad /
              (1.0 + k2_ * theta + k3_ * theta2 + k4_ * theta3 + k5_ * theta4 + k6_ * theta5 + k7_ * theta6);
    }
    const double scaling = std::tan(theta) / thetad;
    x *= scaling;
    y *= scaling;
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Vector3d PolynomialCamera::
cam2world (const Vector2d& uv) const
{
  return cam2world(uv[0], uv[1]);
}

Vector2d PolynomialCamera::
world2cam(const Vector3d& xyz) const
{
  // return world2cam(project2d(xyz));
  Vector2d px;
  if(!distortion_)
  {
    px[0] = fx_*xyz[0] + cx_;
    px[1] = fy_*xyz[1] + cy_;
  }
  else
  {
    double xd, yd;
    const double r = sqrt( xyz( 1 ) * xyz( 1 ) + xyz( 0 ) * xyz( 0 ));
    // if (r < 1e-8)
    // {
    //   return uv;
    // }
    const double theta = acos( xyz( 2 ) / xyz.norm( ) );
    const double thetad = thetad_from_theta(theta);
    const double scaling = thetad / r;
    xd = xyz[0] * scaling;
    yd = xyz[1] * scaling;
    px[0] = xd*fx_ + yd*skew_ + cx_;
    px[1] = yd*fy_ + cy_;
  }
  return px;
}

Vector2d PolynomialCamera::
world2cam(const Vector2d& uv) const
{
  Vector2d px;
  if(!distortion_)
  {
    px[0] = fx_*uv[0] + cx_;
    px[1] = fy_*uv[1] + cy_;
  }
  else
  {
    double xd, yd;
    const double r = uv.norm();
    if (r < 1e-8)
    {
      return uv;
    }
    const double theta = std::atan(r);
    const double thetad = thetad_from_theta(theta);
    const double scaling = thetad / r;
    xd = uv[0] * scaling;
    yd = uv[1] * scaling;
    px[0] = xd*fx_ + yd*skew_ + cx_;
    px[1] = yd*fy_ + cy_;
  }
  return px;
}

} // end namespace vk
