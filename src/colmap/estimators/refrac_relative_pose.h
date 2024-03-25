#pragma once

#include "colmap/geometry/rigid3.h"
#include "colmap/util/eigen_alignment.h"
#include "colmap/util/types.h"

#include <vector>

#include <Eigen/Core>

namespace colmap {
// Solver for the Refractive Relative Pose problem using the approach described
// in the paper:
//
// Hu, Xiao, François Lauze, and Kim Steenstrup Pedersen. "Refractive Pose
// Refinement: Generalising the Geometric Relation between Camera and Refractive
// Interface." International Journal of Computer Vision 131.6 (2023): 1448-1476.
//
// The implementation is a C++ re-implementation based on the original Matlab
// open-source code: https://github.com/diku-dk/RefractiveSfM
class RefracRelPoseEstimator {
 public:
  // The generalized image observations of the left camera, which is composed of
  // the relative transform from the virtual camera to the real camera and a ray
  // in the real camera frame.
  struct X_t {
    Rigid3d virtual_from_real;
    Eigen::Vector3d ray_in_virtual;
  };

  // The normalized image feature points in the right camera.
  typedef X_t Y_t;
  // The estimated cam2_from_cam1 relative pose between the cameras.
  typedef Rigid3d M_t;

  // The minimum number of samples needed to estimate a model. I guess it is 17,
  // not too sure yet.
  static const int kMinNumSamples = 17;

  // Estimate the most probable solution of the refractive relative pose problem
  // from a set of 2D-2D point correspondences.
  static void Estimate(const std::vector<X_t>& points1,
                       const std::vector<Y_t>& points2,
                       std::vector<M_t>* models);

  // Calculate the squared Sampson error between corresponding points.
  static void Residuals(const std::vector<X_t>& points1,
                        const std::vector<Y_t>& points2,
                        const M_t& cam2_from_cam1,
                        std::vector<double>* residuals);
};
}  // namespace colmap
