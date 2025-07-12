/*
 * homography.cpp
 * Adaptation of PTAM-GPL HomographyInit class.
 * https://github.com/Oxford-PTAM/PTAM-GPL
 * Licence: GPLv3
 * Copyright 2008 Isis Innovation Limited
 *
 *  Created on: Sep 2, 2012
 *      by: cforster
 *
 * This class implements the homography decomposition of Faugeras and Lustman's
 * 1988 tech report. Code converted to Eigen from PTAM.
 *
 */

#ifndef HOMOGRAPHY_H_
#define HOMOGRAPHY_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <vikit/math_utils.h>

namespace vk {

using namespace Eigen;
using namespace std;

struct HomographyDecomposition
{
  Vector3d t;
  Matrix3d R;
  double   d;
  Vector3d n;

  // Resolved  Composition
  Sophus::SE3 T; //!< second from first
  int score;
};

class Homography
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Homography            (const vector<Vector2d, aligned_allocator<Vector2d> >& _fts1,
                         const vector<Vector2d, aligned_allocator<Vector2d> >& _fts2,
                         double _error_multiplier2,
                         double _thresh_in_px);

  void
  calcFromPlaneParams   (const Vector3d & normal,
                         const Vector3d & point_on_plane);

  void
  calcFromMatches       ();

  size_t
  computeMatchesInliers ();

  bool
  computeSE3fromMatches ();

  bool
  decompose             ();

  void
  findBestDecomposition ();

  double thresh;
  double error_multiplier2;
  const vector<Vector2d, aligned_allocator<Vector2d> >& fts_c1; //!< Features on first image on unit plane
  const vector<Vector2d, aligned_allocator<Vector2d> >& fts_c2; //!< Features on second image on unit plane
  vector<bool> inliers;
  SE3 T_c2_from_c1;             //!< Relative translation and rotation of two images
  Matrix3d H_c2_from_c1;                   //!< Homography
  vector<HomographyDecomposition> decompositions;
};




} /* end namespace vk */

#endif /* HOMOGRAPHY_H_ */
