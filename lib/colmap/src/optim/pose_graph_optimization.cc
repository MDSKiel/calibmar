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

#include "optim/pose_graph_optimization.h"

#include "base/cost_functions.h"
#include "util/misc.h"
#include "util/threading.h"

namespace colmap {

////////////////////////////////////////////////////////////////////////////////
// PoseGraphOptimizationOptions
////////////////////////////////////////////////////////////////////////////////

ceres::LossFunction* PoseGraphOptimizationOptions::CreateLossFunction() const {
  ceres::LossFunction* loss_function = nullptr;
  switch (loss_function_type) {
    case LossFunctionType::TRIVIAL:
      loss_function = new ceres::TrivialLoss();
      break;
    case LossFunctionType::SOFT_L1:
      loss_function = new ceres::SoftLOneLoss(loss_function_scale);
      break;
    case LossFunctionType::CAUCHY:
      loss_function = new ceres::CauchyLoss(loss_function_scale);
      break;
  }
  CHECK_NOTNULL(loss_function);
  return loss_function;
}

bool PoseGraphOptimizationOptions::Check() const {
  CHECK_OPTION_GE(loss_function_scale, 0);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// PoseGraphOptimizationConfig
////////////////////////////////////////////////////////////////////////////////

PoseGraphOptimizationConfig::PoseGraphOptimizationConfig() {}

size_t PoseGraphOptimizationConfig::NumConstraints() const {
  return edge_constraints_.size();
}

size_t PoseGraphOptimizationConfig::NumConstantPoses() const {
  return constant_poses_.size();
}

void PoseGraphOptimizationConfig::AddEdgeConstraint(
    const PoseGraphEdgeConstraint& constraint) {
  edge_constraints_.push_back(constraint);
}

void PoseGraphOptimizationConfig::AddVertexConstraint(
    const PoseGraphVertexConstraint& constraint) {
  vertex_constraints_.push_back(constraint);
}

const std::vector<PoseGraphEdgeConstraint,
                  Eigen::aligned_allocator<PoseGraphEdgeConstraint>>&
PoseGraphOptimizationConfig::EdgeConstraints() {
  return edge_constraints_;
}

const PoseGraphEdgeConstraint& PoseGraphOptimizationConfig::EdgeConstraint(
    const size_t index) {
  return edge_constraints_[index];
}

const std::vector<PoseGraphVertexConstraint,
                  Eigen::aligned_allocator<PoseGraphVertexConstraint>>&
PoseGraphOptimizationConfig::VertexConstraints() {
  return vertex_constraints_;
}

const PoseGraphVertexConstraint& PoseGraphOptimizationConfig::VertexConstraint(
    const size_t index) {
  return vertex_constraints_[index];
}

void PoseGraphOptimizationConfig::SetConstantPose(const image_t image_id) {
  constant_poses_.insert(image_id);
}

void PoseGraphOptimizationConfig::SetVariablePose(const image_t image_id) {
  constant_poses_.erase(image_id);
}

bool PoseGraphOptimizationConfig::HasConstantPose(const image_t image_id) {
  return constant_poses_.find(image_id) != constant_poses_.end();
}

////////////////////////////////////////////////////////////////////////////////
// PoseGraphOptimizer
////////////////////////////////////////////////////////////////////////////////

PoseGraphOptimizer::PoseGraphOptimizer(
    const PoseGraphOptimizationOptions& options,
    const PoseGraphOptimizationConfig& config)
    : options_(options), config_(config) {
  CHECK(options_.Check());
}

bool PoseGraphOptimizer::Solve(Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);
  CHECK(!problem_) << "Cannot use the same PoseGraphOptimizer multiple times";

  problem_.reset(new ceres::Problem());

  ceres::LossFunction* loss_function = options_.CreateLossFunction();
  SetUp(reconstruction, loss_function);

  if (problem_->NumResiduals() == 0) {
    return false;
  }

  ceres::Solver::Options solver_options = options_.solver_options;
  solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  if (problem_->NumResiduals() <
      options_.min_num_residuals_for_multi_threading) {
    solver_options.num_threads = 1;
#if CERES_VERSION_MAJOR < 2
    solver_options.num_linear_solver_threads = 1;
#endif  // CERES_VERSION_MAJOR
  } else {
    solver_options.num_threads =
        GetEffectiveNumThreads(solver_options.num_threads);
#if CERES_VERSION_MAJOR < 2
    solver_options.num_linear_solver_threads =
        GetEffectiveNumThreads(solver_options.num_linear_solver_threads);
#endif  // CERES_VERSION_MAJOR
  }

  std::string solver_error;
  CHECK(solver_options.IsValid(&solver_error)) << solver_error;

  ceres::Solve(solver_options, problem_.get(), &summary_);

  if (solver_options.minimizer_progress_to_stdout) {
    std::cout << std::endl;
  }

  if (options_.print_summary) {
    PrintHeading2("Pose graph optimization report");
    PrintSolverSummary(summary_);
  }

  TearDown(reconstruction);

  return true;
}

const ceres::Solver::Summary& PoseGraphOptimizer::Summary() const {
  return summary_;
}

void PoseGraphOptimizer::SetUp(Reconstruction* reconstruction,
                               ceres::LossFunction* loss_function) {
  const std::vector<PoseGraphVertexConstraint,
                    Eigen::aligned_allocator<PoseGraphVertexConstraint>>
      vertex_constraints = config_.VertexConstraints();
  const std::vector<PoseGraphEdgeConstraint,
                    Eigen::aligned_allocator<PoseGraphEdgeConstraint>>&
      edge_constraints = config_.EdgeConstraints();
  for (const PoseGraphVertexConstraint& constraint : vertex_constraints) {
    AddVertexConstraintToProblem(constraint, reconstruction, loss_function);
  }
  for (const PoseGraphEdgeConstraint& constraint : edge_constraints) {
    AddEdgeConstraintToProblem(constraint, reconstruction, loss_function);
  }
}

void PoseGraphOptimizer::AddEdgeConstraintToProblem(
    const PoseGraphEdgeConstraint& constraint, Reconstruction* reconstruction,
    ceres::LossFunction* loss_function) {
  Image& image1 = reconstruction->Image(constraint.image_id1);
  Image& image2 = reconstruction->Image(constraint.image_id2);

  image1.NormalizeQvec();
  image2.NormalizeQvec();

  double* qvec1_data = image1.Qvec().data();
  double* tvec1_data = image1.Tvec().data();
  double* qvec2_data = image2.Qvec().data();
  double* tvec2_data = image2.Tvec().data();

  const bool constant_pose1 = config_.HasConstantPose(constraint.image_id1);
  const bool constant_pose2 = config_.HasConstantPose(constraint.image_id2);

  const Eigen::Matrix<double, 6, 6> sqrt_information =
      constraint.information.llt().matrixL();

  ceres::CostFunction* cost_function = nullptr;
  switch (constraint.type) {
    case PoseGraphEdgeConstraint::POSE_PRIOR:
      cost_function = RelativePose6DoFCostFunction::Create(
          constraint.qvec12, constraint.tvec12, sqrt_information);
      break;
    case PoseGraphEdgeConstraint::TWO_VIEW_GEOMETRY:
      cost_function = RelativePose5DoFCostFunction::Create(
          constraint.qvec12, constraint.tvec12, sqrt_information);
      break;
  }

  problem_->AddResidualBlock(cost_function, loss_function, qvec1_data,
                             tvec1_data, qvec2_data, tvec2_data);

  // Set pose parameterization.
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;
  if (constant_pose1) {
    problem_->SetParameterBlockConstant(qvec1_data);
    problem_->SetParameterBlockConstant(tvec1_data);
  } else {
    problem_->SetParameterization(qvec1_data, quaternion_parameterization);
  }
  if (constant_pose2) {
    problem_->SetParameterBlockConstant(qvec2_data);
    problem_->SetParameterBlockConstant(tvec2_data);
  } else {
    problem_->SetParameterization(qvec2_data, quaternion_parameterization);
  }
}

void PoseGraphOptimizer::AddVertexConstraintToProblem(
    const PoseGraphVertexConstraint& constraint, Reconstruction* reconstruction,
    ceres::LossFunction* loss_function) {
  Image& image = reconstruction->Image(constraint.image_id);

  image.NormalizeQvec();

  double* qvec_data = image.Qvec().data();
  double* tvec_data = image.Tvec().data();

  const bool constant_pose = config_.HasConstantPose(constraint.image_id);

  const Eigen::Matrix<double, 6, 6> sqrt_information =
      constraint.information.llt().matrixL();

  ceres::CostFunction* cost_function = AbsolutePose6DoFCostFunction::Create(
      image.QvecPrior(), image.TvecPrior(), sqrt_information);
  problem_->AddResidualBlock(cost_function, loss_function, qvec_data,
                             tvec_data);

  // Set pose parameterization.
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;
  if (constant_pose) {
    problem_->SetParameterBlockConstant(qvec_data);
    problem_->SetParameterBlockConstant(tvec_data);
  } else {
    problem_->SetParameterization(qvec_data, quaternion_parameterization);
  }
}

void PoseGraphOptimizer::TearDown(Reconstruction*) {
  // Nothing to do
}

}  // namespace colmap