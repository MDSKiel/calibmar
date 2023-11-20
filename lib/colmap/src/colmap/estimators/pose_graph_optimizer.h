#pragma once

#include "colmap/geometry/rigid3.h"
#include "colmap/scene/reconstruction.h"

#include <memory>
#include <unordered_set>

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace colmap {

class PoseGraphOptimizer {
 public:
  PoseGraphOptimizer(const std::shared_ptr<Reconstruction>& reconstruction);

  void AddAbsolutePose(image_t image_id,
                       const Rigid3d& tform_measured,
                       const Eigen::Matrix6d& information,
                       ceres::LossFunction* loss_function);

  void AddRelativePose(image_t image_id1,
                       image_t image_id2,
                       const Rigid3d& cam2_from_cam1_measured,
                       const Eigen::Matrix6d& information,
                       ceres::LossFunction* loss_function);

  void AddSmoothMotion(image_t image_id1,
                       image_t image_id2,
                       image_t image_id3,
                       const Eigen::Matrix6d& information,
                       ceres::LossFunction* loss_function);
  bool Solve();

 protected:
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Solver::Summary summary_;
  std::shared_ptr<Reconstruction> reconstruction_;
};

}  // namespace colmap