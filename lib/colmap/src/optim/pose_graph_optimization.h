// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_OPTIM_POSE_GRAPH_OPTIMIZATION_H_
#define COLMAP_SRC_OPTIM_POSE_GRAPH_OPTIMIZATION_H_

#include "optim/bundle_adjustment.h"

namespace colmap {

// Pose graph edge constraint.
struct PoseGraphEdgeConstraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The type of the pose graph constraint.
  enum ConstraintType {
    // Pose prior constraint.
    POSE_PRIOR = 0,

    // Two-view geometry constraint.
    TWO_VIEW_GEOMETRY = 1,
  };

  // One of the ConstraintType
  ConstraintType type;

  // Image ID for image 1 and image 2.
  image_t image_id1;
  image_t image_id2;

  // Relative transformation from pose 1 to pose 2.
  Eigen::Vector4d qvec12;
  Eigen::Vector3d tvec12;

  // Information matrix for the pose constraint.
  Eigen::Matrix<double, 6, 6> information;
};

// Pose graph vertex constraint.
struct PoseGraphVertexConstraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Image ID
  image_t image_id;

  // Pose constraint.
  Eigen::Vector4d qvec;
  Eigen::Vector3d tvec;

  // Information matrix for the pose constraint.
  Eigen::Matrix<double, 6, 6> information;
};

struct PoseGraphOptimizationOptions {
  // Loss function types: Trivial (non-robust) and Cauchy (robust) loss.
  enum class LossFunctionType { TRIVIAL, SOFT_L1, CAUCHY };
  LossFunctionType loss_function_type = LossFunctionType::TRIVIAL;

  // Scaling factor determines residual at which robustfication takes place.
  double loss_function_scale = 1.0;

  /// TODO: (add a switch) Whether to use pose priors (requires pose priors).
  // bool use_pose_priors = true;

  /// TODO: (add a switch) Whether to use two-view geometry.
  // bool use_two_view_geometry = true;

  // Whether to print a final summary.
  bool print_summary = true;

  // Minimum number of residuals to enable multi-threading. Note that
  // single-threaded is typically better for small bundle adjustment problems
  // due to the overhead of threading.
  int min_num_residuals_for_multi_threading = 50000;

  ceres::Solver::Options solver_options;

  PoseGraphOptimizationOptions() {
    solver_options.function_tolerance = 0.0;
    solver_options.gradient_tolerance = 0.0;
    solver_options.parameter_tolerance = 0.0;
    solver_options.minimizer_progress_to_stdout = false;
    solver_options.max_num_iterations = 100;
    solver_options.max_linear_solver_iterations = 200;
    solver_options.max_num_consecutive_invalid_steps = 10;
    solver_options.max_consecutive_nonmonotonic_steps = 10;
    solver_options.num_threads = -1;
#if CERES_VERSION_MAJOR < 2
    solver_options.num_linear_solver_threads = -1;
#endif  // CERES_VERSION_MAJOR
  }

  // Create a new loss function based on the specified options. The caller
  // takes ownership of the loss function.
  ceres::LossFunction* CreateLossFunction() const;

  bool Check() const;
};

// Configuration container to setup pose graph optimization problems.
class PoseGraphOptimizationConfig {
 public:
  PoseGraphOptimizationConfig();

  size_t NumConstraints() const;
  size_t NumConstantPoses() const;

  // Add pose graph edge constraint.
  void AddEdgeConstraint(const PoseGraphEdgeConstraint& constraint);

  // Add pose graph vertex constraint.
  void AddVertexConstraint(const PoseGraphVertexConstraint& constraint);

  // Access const edge constraint data.
  const std::vector<PoseGraphEdgeConstraint,
                    Eigen::aligned_allocator<PoseGraphEdgeConstraint>>&
  EdgeConstraints();
  const PoseGraphEdgeConstraint& EdgeConstraint(const size_t index);

  // Access const vertex constraint data.
  const std::vector<PoseGraphVertexConstraint,
                    Eigen::aligned_allocator<PoseGraphVertexConstraint>>&
  VertexConstraints();
  const PoseGraphVertexConstraint& VertexConstraint(const size_t index);

  // Set the pose of added images as constant. The pose is defined as the
  // rotational and translational part of the projection matrix.
  void SetConstantPose(const image_t image_id);
  void SetVariablePose(const image_t image_id);
  bool HasConstantPose(const image_t image_id);

 private:
  std::vector<PoseGraphEdgeConstraint,
              Eigen::aligned_allocator<PoseGraphEdgeConstraint>>
      edge_constraints_;
  std::vector<PoseGraphVertexConstraint,
              Eigen::aligned_allocator<PoseGraphVertexConstraint>>
      vertex_constraints_;
  std::unordered_set<image_t> constant_poses_;
};

// Pose graph optimization based on Ceres-Solver.
class PoseGraphOptimizer {
 public:
  PoseGraphOptimizer(const PoseGraphOptimizationOptions& options,
                     const PoseGraphOptimizationConfig& config);

  bool Solve(Reconstruction* reconstruction);

  // Get the Ceres Solver summary for the last call to `Solve`.
  const ceres::Solver::Summary& Summary() const;

 private:
  void SetUp(Reconstruction* reconstruction,
             ceres::LossFunction* loss_function);

  void AddEdgeConstraintToProblem(const PoseGraphEdgeConstraint& constraint,
                                  Reconstruction* reconstruction,
                                  ceres::LossFunction* loss_function);
  void AddVertexConstraintToProblem(const PoseGraphVertexConstraint& constraint,
                                    Reconstruction* reconstruction,
                                    ceres::LossFunction* loss_function);

  void TearDown(Reconstruction* reconstruction);

 protected:
  const PoseGraphOptimizationOptions options_;
  PoseGraphOptimizationConfig config_;
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Solver::Summary summary_;
};

}  // namespace colmap

#endif  // COLMAP_SRC_OPTIM_POSE_GRAPH_OPTIMIZATION_H_