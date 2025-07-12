/*
 * file_reader_types.h
 *
 *  Created on: Jul 1, 2014
 *      Author: cforster
 */

#ifndef VIKIT_FILE_READER_TYPES_H_
#define VIKIT_FILE_READER_TYPES_H_

namespace vk {

/// Common types
namespace file_format {

/// IMU rotational velocity and linear acceleration
class ImuRotvelLinacc
{
public:
  ImuRotvelLinacc() {}
  virtual ~ImuRotvelLinacc() {}
  double timestamp_;            //!< timestamp in seconds
  Eigen::Vector3d w_;           //!< angular velocity
  Eigen::Vector3d a_;        //!< linear acceleration
  friend std::ostream& operator <<(std::ostream& out, const ImuRotvelLinacc& pair);
  friend std::istream& operator >>(std::istream& in, ImuRotvelLinacc& pair);
};

std::ostream& operator <<(std::ostream& out, const ImuRotvelLinacc& gt)
{
  out << gt.timestamp_ << " "
      << gt.w_.x() << " " << gt.w_.y() << " " << gt.w_.z() << " "
      << gt.a_.x() << " " << gt.a_.y() << " " << gt.a_.z() << std::endl;
  return out;
}

std::istream& operator >>(std::istream& in, ImuRotvelLinacc& gt)
{
  double wx, wy, wz, ax, ay, az;
  in >> gt.timestamp_;
  in >> wx;
  in >> wy;
  in >> wz;
  in >> ax;
  in >> ay;
  in >> az;
  gt.w_ = Eigen::Vector3d(wx, wy, wz);
  gt.a_ = Eigen::Vector3d(ax, ay, az);
  return in;
}


/// Timestamp with Position and Orientation
class PoseStamped
{
public:
  PoseStamped() {}
  virtual ~PoseStamped() {}
  double timestamp_;            //!< timestamp in seconds
  Eigen::Vector3d t_;           //!< position
  Eigen::Quaterniond q_;        //!< orientation
  friend std::ostream& operator <<(std::ostream& out, const ImuRotvelLinacc& pair);
  friend std::istream& operator >>(std::istream& in, ImuRotvelLinacc& pair);
};

std::ostream& operator <<(std::ostream& out, const PoseStamped& gt)
{
  out << gt.timestamp_ << " "
      << gt.t_.x() << " " << gt.t_.y() << " " << gt.t_.z() << " "
      << gt.q_.x() << " " << gt.q_.y() << " " << gt.q_.z() << " " << gt.q_.w()<< " "
      << std::endl;
  return out;
}

std::istream& operator >>(std::istream& in, PoseStamped& gt)
{
  in >> gt.timestamp_;
  double tx, ty, tz, qx, qy, qz, qw;
  in >> tx;
  in >> ty;
  in >> tz;
  in >> qx;
  in >> qy;
  in >> qz;
  in >> qw;
  gt.t_ = Eigen::Vector3d(tx, ty, tz);
  gt.q_ = Eigen::Quaterniond(qw, qx, qy, qz);
  gt.q_.normalize();
  return in;
}

} // namespace file_format
} // namespace vk

#endif // VIKIT_FILE_READER_TYPES_H_
