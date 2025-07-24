/*
 * equidistant_camera.h
 *
 *  Created on: January 26, 2023
 *      Author: xuankuzcr
 */

#ifndef EQUIDISTANT_CAMERA_H_
#define EQUIDISTANT_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>

namespace vk {

using namespace std;
using namespace Eigen;

class EquidistantCamera : public AbstractCamera {

private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;             //!< is it pure pinhole model or has it equidistant distortion
  double k1_, k2_, k3_, k4_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EquidistantCamera(double width, double height, double scale,
                double fx, double fy, double cx, double cy,
                double k1=0.0, double k2=0.0, double k3=0.0, double k4=0.0);

  ~EquidistantCamera();

  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Vector3d
  cam2world(const Vector2d& px) const;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }
  
  inline double thetad_from_theta(const double theta) const
  {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double thetad = theta * (1.0 + k1_ * theta2 + k2_ * theta4 +
                                   k3_ * theta6 + k4_ * theta8);
    return thetad;
  }

  inline double deriv_thetad_from_theta(const double theta) const
  {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    return 1 + 3 * k1_ * theta2 + 5 * k2_ * theta4 + 7 * k3_ * theta6 +
           9 * k4_ * theta8;
  }

  inline Eigen::Matrix2d jacobian_2x2(const Eigen::Vector2d& uv) const
  {
    const double r = uv.norm();
    if (r < 1e-8)
    {
      return Eigen::Matrix2d::Identity();
    }

    const double inv_r = 1.0 / r;
    const double r2 = r * r;
    const double dr_du = uv(0) * inv_r;
    const double dr_dv = uv(1) * inv_r;

    const double theta = std::atan(r);
    const double dtheta_dr = 1.0 / (1 + r * r);

    const double thetad = thetad_from_theta(theta);
    const double dthetad_dtheta = deriv_thetad_from_theta(theta);
    const double dthetad_dr = dthetad_dtheta * dtheta_dr;

    const double scaling = thetad / r;
    const double dscaling_du = (dthetad_dr * dr_du * r - dr_du * thetad) / r2;
    const double dscaling_dv = (dthetad_dr * dr_dv * r - dr_dv * thetad) / r2;

    const double dx_du = dscaling_du * uv(0) + scaling;
    const double dx_dv = dscaling_dv * uv(0);
    const double dy_du = dscaling_du * uv(1);
    const double dy_dv = dscaling_dv * uv(1) + scaling;
    Eigen::Matrix2d jac;
    jac << dx_du, dx_dv, dy_du, dy_dv;
    return jac;
  }

  inline Eigen::Matrix<double, 2, 3> jacobian_2x3(const Eigen::Vector3d& p) const
  {
    Eigen::Matrix<double, 2, 3> jac;
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1./p[2];
    const double z_inv_2 = z_inv * z_inv;
    jac(0,0) = fx_ * z_inv;
    jac(0,1) = 0.0;
    jac(0,2) = -fx_ * x * z_inv_2;
    jac(1,0) = 0.0;
    jac(1,1) = fy_ * z_inv;
    jac(1,2) = -fy_ * y * z_inv_2;
    return jac;
  }

  virtual double errorMultiplier2() const
  {
    return fabs(fx_);
  }

  virtual double errorMultiplier() const
  {
    return fabs(4.0*fx_*fy_);
  }

  virtual double fx() const { return fx_; };
  virtual double fy() const { return fy_; };
  virtual double cx() const { return cx_; };
  virtual double cy() const { return cy_; };
};

} // end namespace vk


#endif /* #define EQUIDISTANT_CAMERA_H_ */
