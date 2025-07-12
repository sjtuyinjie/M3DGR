/*
 * blender_utils.h
 *
 *  Created on: Feb 13, 2014
 *      Author: cforster
 */

#ifndef VIKIT_BLENDER_UTILS_H_
#define VIKIT_BLENDER_UTILS_H_

#include <list>
#include <string>
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Eigen/Core>

namespace vk {
namespace blender_utils {

void loadBlenderDepthmap(
    const std::string file_name,
    const vk::AbstractCamera& cam,
    cv::Mat& img)
{
  std::ifstream file_stream(file_name.c_str());
  assert(file_stream.is_open());
  img = cv::Mat(cam.height(), cam.width(), CV_32FC1);
  float * img_ptr = img.ptr<float>();
  float depth;
  for(int y=0; y<cam.height(); ++y)
  {
    for(int x=0; x<cam.width(); ++x, ++img_ptr)
    {
      file_stream >> depth;
      // blender:
      Eigen::Vector2d uv(vk::project2d(cam.cam2world(x,y)));
      *img_ptr = depth * sqrt(uv[0]*uv[0] + uv[1]*uv[1] + 1.0);

      // povray
      // *img_ptr = depth/100.0; // depth is in [cm], we want [m]

      if(file_stream.peek() == '\n' && x != cam.width()-1 && y != cam.height()-1)
        printf("WARNING: did not read the full depthmap!\n");
    }
  }
}

bool getDepthmapNormalAtPoint(
    const Vector2i& px,
    const cv::Mat& depth,
    const int halfpatch_size,
    const vk::AbstractCamera& cam,
    Vector3d& normal)
{
  assert(cam.width() == depth.cols && cam.height() == depth.rows);
  if(!cam.isInFrame(px, halfpatch_size+1))
    return false;

  const size_t n_meas = (halfpatch_size*2+1)*(halfpatch_size*2+1);
  list<Vector3d> pts;
  for(int y = px[1]-halfpatch_size; y<=px[1]+halfpatch_size; ++y)
    for(int x = px[0]-halfpatch_size; x<=px[0]+halfpatch_size; ++x)
      pts.push_back(cam.cam2world(x,y)*depth.at<float>(y,x));

  assert(n_meas == pts.size());
  Matrix<double, Dynamic, 4> A; A.resize(n_meas, Eigen::NoChange);
  Matrix<double, Dynamic, 1> b; b.resize(n_meas, Eigen::NoChange);

  size_t i = 0;
  for(list<Vector3d>::iterator it=pts.begin(); it!=pts.end(); ++it)
  {
    A.row(i) << it->x(), it->y(), it->z(), 1.0;
    b[i] = 0;
    ++i;
  }

  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

  Matrix<double, 4, 4> V = svd.matrixV();
  normal = V.block<3,1>(0,3);
  normal.normalize();
  return true;
}

namespace file_format
{

class ImageNameAndPose
{
public:
  ImageNameAndPose() {}
  virtual ~ImageNameAndPose() {}
  double timestamp_;
  std::string image_name_;
  Eigen::Vector3d t_;
  Eigen::Quaterniond q_;
  friend std::ostream& operator <<(std::ostream& out, const ImageNameAndPose& pair);
  friend std::istream& operator >>(std::istream& in, ImageNameAndPose& pair);
};

std::ostream& operator <<(std::ostream& out, const ImageNameAndPose& gt)
{
  out << gt.timestamp_ << " " << gt.image_name_ << " "
      << gt.t_.x()   << " " << gt.t_.y()   << " " << gt.t_.z()   << " "
      << gt.q_.x()   << " " << gt.q_.y()   << " " << gt.q_.z()   << " " << gt.q_.w()   << " " << std::endl;
  return out;
}

std::istream& operator >>(std::istream& in, ImageNameAndPose& gt)
{
  in >> gt.timestamp_;
  in >> gt.image_name_;
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
} // namespace blender_utils
} // namespace vk

#endif // VIKIT_BLENDER_UTILS_H_
