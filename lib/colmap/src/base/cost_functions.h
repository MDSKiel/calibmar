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

#ifndef COLMAP_SRC_BASE_COST_FUNCTIONS_H_
#define COLMAP_SRC_BASE_COST_FUNCTIONS_H_

#include <Eigen/Core>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "base/ray3d.h"

namespace colmap {

// Compute the 6-DoF absolute difference between pose1 and pose2.
// The error has 6 dimensions as the following:
// [ translation residuals];
// [ orientation (3x1) reisduals]
//
// ref pose_graph_3d from Ceres-Solver:
// http://ceres-solver.org/nnls_tutorial.html#other-examples
template <typename T>
void ComputePose6DoFError(const T* const qvec1, const T* const tvec1,
                          const T* const qvec2, const T* const tvec2, T* error);

// Compute the rotation difference between qvec 1 and qvec 2.
template <typename T>
void ComputeRotation3DError(const T* const qvec1, const T* const qvec2,
                            T* error);

// Standard bundle adjustment cost function for variable
// camera pose and calibration and point parameters.
template <typename CameraModel>
class BundleAdjustmentCostFunction {
 public:
  explicit BundleAdjustmentCostFunction(const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentCostFunction<CameraModel>, 2, 4, 3, 3,
            CameraModel::kNumParams>(
        new BundleAdjustmentCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    // Rotate and translate.
    T projection[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, projection);
    projection[0] += tvec[0];
    projection[1] += tvec[1];
    projection[2] += tvec[2];

    // Project to image plane.
    projection[0] /= projection[2];
    projection[1] /= projection[2];

    // Distort and transform to pixel space.
    CameraModel::WorldToImage(camera_params, projection[0], projection[1],
                              &residuals[0], &residuals[1]);

    // Re-projection error.
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
};

// Bundle adjustment cost function for variable
// camera calibration and point parameters, and fixed camera pose.
template <typename CameraModel>
class BundleAdjustmentConstantPoseCostFunction {
 public:
  BundleAdjustmentConstantPoseCostFunction(const Eigen::Vector4d& qvec,
                                           const Eigen::Vector3d& tvec,
                                           const Eigen::Vector2d& point2D)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        observed_x_(point2D(0)),
        observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                     const Eigen::Vector3d& tvec,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentConstantPoseCostFunction<CameraModel>, 2, 3,
            CameraModel::kNumParams>(
        new BundleAdjustmentConstantPoseCostFunction(qvec, tvec, point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    const T qvec[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};

    // Rotate and translate.
    T projection[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, projection);
    projection[0] += T(tx_);
    projection[1] += T(ty_);
    projection[2] += T(tz_);

    // Project to image plane.
    projection[0] /= projection[2];
    projection[1] /= projection[2];

    // Distort and transform to pixel space.
    CameraModel::WorldToImage(camera_params, projection[0], projection[1],
                              &residuals[0], &residuals[1]);

    // Re-projection error.
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const double observed_x_;
  const double observed_y_;
};

// Standard bundle adjustment cost function for variable
// camera pose and calibration and constant point parameters.
template <typename CameraModel>
class BundleAdjustmentConstantPoint3DCostFunction {
 public:
  explicit BundleAdjustmentConstantPoint3DCostFunction(
      const Eigen::Vector2d& point2D, const Eigen::Vector3d& point3D)
      : observed_x_(point2D(0)),
        observed_y_(point2D(1)),
        point3D_x_(point3D(0)),
        point3D_y_(point3D(1)),
        point3D_z_(point3D(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const Eigen::Vector3d& point3D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentConstantPoint3DCostFunction<CameraModel>, 2, 4, 3,
            CameraModel::kNumParams>(
        new BundleAdjustmentConstantPoint3DCostFunction(point2D, point3D)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const camera_params, T* residuals) const {
    const T point3D[3] = {T(point3D_x_), T(point3D_y_), T(point3D_z_)};

    // Rotate and translate.
    T projection[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, projection);
    projection[0] += tvec[0];
    projection[1] += tvec[1];
    projection[2] += tvec[2];

    // Project to image plane.
    projection[0] /= projection[2];
    projection[1] /= projection[2];

    // Distort and transform to pixel space.
    CameraModel::WorldToImage(camera_params, projection[0], projection[1],
                              &residuals[0], &residuals[1]);

    // Re-projection error.
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
  const double point3D_x_;
  const double point3D_y_;
  const double point3D_z_;
};

// Rig bundle adjustment cost function for variable camera pose and calibration
// and point parameters. Different from the standard bundle adjustment function,
// this cost function is suitable for camera rigs with consistent relative poses
// of the cameras within the rig. The cost function first projects points into
// the local system of the camera rig and then into the local system of the
// camera within the rig.
template <typename CameraModel>
class RigBundleAdjustmentCostFunction {
 public:
  explicit RigBundleAdjustmentCostFunction(const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            RigBundleAdjustmentCostFunction<CameraModel>, 2, 4, 3, 4, 3, 3,
            CameraModel::kNumParams>(
        new RigBundleAdjustmentCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const rig_qvec, const T* const rig_tvec,
                  const T* const rel_qvec, const T* const rel_tvec,
                  const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    // Concatenate rotations.
    T qvec[4];
    ceres::QuaternionProduct(rel_qvec, rig_qvec, qvec);

    // Concatenate translations.
    T tvec[3];
    ceres::UnitQuaternionRotatePoint(rel_qvec, rig_tvec, tvec);
    tvec[0] += rel_tvec[0];
    tvec[1] += rel_tvec[1];
    tvec[2] += rel_tvec[2];

    // Rotate and translate.
    T projection[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, projection);
    projection[0] += tvec[0];
    projection[1] += tvec[1];
    projection[2] += tvec[2];

    // Project to image plane.
    projection[0] /= projection[2];
    projection[1] /= projection[2];

    // Distort and transform to pixel space.
    CameraModel::WorldToImage(camera_params, projection[0], projection[1],
                              &residuals[0], &residuals[1]);

    // Re-projection error.
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
};

// Cost function for refining two-view geometry based on the Sampson-Error.
//
// First pose is assumed to be located at the origin with 0 rotation. Second
// pose is assumed to be on the unit sphere around the first pose, i.e. the
// pose of the second camera is parameterized by a 3D rotation and a
// 3D translation with unit norm. `tvec` is therefore over-parameterized as is
// and should be down-projected using `HomogeneousVectorParameterization`.
class RelativePoseCostFunction {
 public:
  RelativePoseCostFunction(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2)
      : x1_(x1(0)), y1_(x1(1)), x2_(x2(0)), y2_(x2(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& x1,
                                     const Eigen::Vector2d& x2) {
    return (new ceres::AutoDiffCostFunction<RelativePoseCostFunction, 1, 4, 3>(
        new RelativePoseCostFunction(x1, x2)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  T* residuals) const {
    Eigen::Matrix<T, 3, 3, Eigen::RowMajor> R;
    ceres::QuaternionToRotation(qvec, R.data());

    // Matrix representation of the cross product t x R.
    Eigen::Matrix<T, 3, 3> t_x;
    t_x << T(0), -tvec[2], tvec[1], tvec[2], T(0), -tvec[0], -tvec[1], tvec[0],
        T(0);

    // Essential matrix.
    const Eigen::Matrix<T, 3, 3> E = t_x * R;

    // Homogeneous image coordinates.
    const Eigen::Matrix<T, 3, 1> x1_h(T(x1_), T(y1_), T(1));
    const Eigen::Matrix<T, 3, 1> x2_h(T(x2_), T(y2_), T(1));

    // Squared sampson error.
    const Eigen::Matrix<T, 3, 1> Ex1 = E * x1_h;
    const Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * x2_h;
    const T x2tEx1 = x2_h.transpose() * Ex1;
    residuals[0] = x2tEx1 * x2tEx1 /
                   (Ex1(0) * Ex1(0) + Ex1(1) * Ex1(1) + Etx2(0) * Etx2(0) +
                    Etx2(1) * Etx2(1));

    return true;
  }

 private:
  const double x1_;
  const double y1_;
  const double x2_;
  const double y2_;
};

// Standard bundle adjustment cost function for camera parameters only.
// Camera poses and points are constant
template <typename CameraModel>
class BundleAdjustmentConstantPoseConstantPoint3DCostFunction {
 public:
  BundleAdjustmentConstantPoseConstantPoint3DCostFunction(
      const Eigen::Vector2d& point2D, const Eigen::Vector3d& point3D,
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec)
      : observed_x_(point2D(0)),
        observed_y_(point2D(1)),
        point3D_x_(point3D(0)),
        point3D_y_(point3D(1)),
        point3D_z_(point3D(2)),
        qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const Eigen::Vector3d& point3D,
                                     const Eigen::Vector4d& qvec,
                                     const Eigen::Vector3d& tvec) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentConstantPoseConstantPoint3DCostFunction<
                CameraModel>,
            2, CameraModel::kNumParams>(
        new BundleAdjustmentConstantPoseConstantPoint3DCostFunction(
            point2D, point3D, qvec, tvec)));
  }

  template <typename T>
  bool operator()(const T* const camera_params, T* residuals) const {
    const T qvec[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};

    const T point3D[3] = {T(point3D_x_), T(point3D_y_), T(point3D_z_)};

    // Rotate and translate.
    T projection[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, projection);
    projection[0] += T(tx_);
    projection[1] += T(ty_);
    projection[2] += T(tz_);

    // Project to image plane.
    projection[0] /= projection[2];
    projection[1] /= projection[2];

    // Distort and transform to pixel space.
    CameraModel::WorldToImage(camera_params, projection[0], projection[1],
                              &residuals[0], &residuals[1]);

    // Re-projection error.
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
  const double point3D_x_;
  const double point3D_y_;
  const double point3D_z_;
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
};

// Standard bundle adjustment cost function for
// Non-Single-View-Point cameras using reprojection error
// Variables include camera poses and calibration and point parameters
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraCostFunction {
 public:
  explicit BundleAdjustmentNonSvpCameraCostFunction(
      const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraCostFunction<CameraNonSvpModel,
                                                     CameraModel>,
            2, 4, 3, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* qvec, const T* tvec, const T* const point3D,
                  const T* const camera_params, const T* const non_svp_params,
                  T* residuals) const {
    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += tvec[0];
    point3D_camera[1] += tvec[1];
    point3D_camera[2] += tvec[2];

    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const world_point(point3D_camera);

    // Project to image plane.
    T projection[2];
    CameraNonSvpModel::template WorldToImage<CameraModel, T>(
        non_svp_params, camera_params, world_point, &projection[0],
        &projection[1]);

    residuals[0] = projection[0] - T(observed_x_);
    residuals[1] = projection[1] - T(observed_y_);
    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
};

// Standard bundle adjustment cost function for
// Non-Single-View-Point cameras using back-projected 3D error function
// Variables include camera poses and calibration and point parameters
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraCostFunctionBackProjection {
 public:
  explicit BundleAdjustmentNonSvpCameraCostFunctionBackProjection(
      const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraCostFunctionBackProjection<
                CameraNonSvpModel, CameraModel>,
            3, 4, 3, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraCostFunctionBackProjection(point2D)));
  }

  template <typename T>
  bool operator()(const T* qvec, const T* tvec, const T* const point3D,
                  const T* const camera_params, const T* const non_svp_params,
                  T* residuals) const {
    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += tvec[0];
    point3D_camera[1] += tvec[1];
    point3D_camera[2] += tvec[2];

    // Compute depth of the 3D point to the camera center
    const T depth = ceres::sqrt(point3D_camera[0] * point3D_camera[0] +
                                point3D_camera[1] * point3D_camera[1] +
                                point3D_camera[2] * point3D_camera[2]);

    // Compute back-projected 3D point given depth
    Eigen::Matrix<T, 3, 1> back_projection;
    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, T(observed_x_), T(observed_y_), depth,
        &back_projection);

    residuals[0] = back_projection[0] - point3D_camera[0];
    residuals[1] = back_projection[1] - point3D_camera[1];
    residuals[2] = back_projection[2] - point3D_camera[2];

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
};

// Standard bundle adjustment cost function for
// Non-Single-View-Point cameras using back-projected ray-point distance error
// Variables include camera poses and calibration and point parameters
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraCostFunctionPointRayDistance {
 public:
  explicit BundleAdjustmentNonSvpCameraCostFunctionPointRayDistance(
      const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraCostFunctionPointRayDistance<
                CameraNonSvpModel, CameraModel>,
            1, 4, 3, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraCostFunctionPointRayDistance(point2D)));
  }

  template <typename T>
  bool operator()(const T* qvec, const T* tvec, const T* const point3D,
                  const T* const camera_params, const T* const non_svp_params,
                  T* residuals) const {
    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += tvec[0];
    point3D_camera[1] += tvec[1];
    point3D_camera[2] += tvec[2];

    // Compute back-projected 3D ray
    Eigen::Matrix<T, 3, 1> ray_ori = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> ray_dir = Eigen::Matrix<T, 3, 1>::Zero();
    ray_dir(2) = T(1.0);
    CameraNonSvpModel::template ImageToWorld<CameraModel, T>(
        non_svp_params, camera_params, T(observed_x_), T(observed_y_), &ray_ori,
        &ray_dir);

    // Compute point-ray distance
    const Eigen::Matrix<T, 3, 1> point(point3D_camera[0], point3D_camera[1],
                                       point3D_camera[2]);
    residuals[0] = PointToRayDistance(point, ray_ori, ray_dir);

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
};

// Bundle adjustment cost function for Non-Single-View-Point cameras
// using reprojection error function.
// Variables include camera calibration, non-SVP camera parameters and point
// parameters
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraConstantPoseCostFunction {
 public:
  explicit BundleAdjustmentNonSvpCameraConstantPoseCostFunction(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Vector2d& point2D)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        observed_x_(point2D(0)),
        observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                     const Eigen::Vector3d& tvec,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraConstantPoseCostFunction<
                CameraNonSvpModel, CameraModel>,
            3, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraConstantPoseCostFunction(qvec, tvec,
                                                                 point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  const T* const non_svp_params, T* residuals) const {
    const T qvec[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};

    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += T(tx_);
    point3D_camera[1] += T(ty_);
    point3D_camera[2] += T(tz_);

    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const world_point(point3D_camera);

    // Project to image plane.
    T projection[2];
    CameraNonSvpModel::template WorldToImage<CameraModel, T>(
        non_svp_params, camera_params, world_point, &projection[0],
        &projection[1]);

    residuals[0] = projection[0] - T(observed_x_);
    residuals[1] = projection[1] - T(observed_y_);
    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const double observed_x_;
  const double observed_y_;
};

// Bundle adjustment cost function for Non-Single-View-Point cameras
// using back-projected 3D error function.
// Variables include camera calibration, non-SVP camera parameters and point
// parameters
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraConstantPoseCostFunctionBackProjection {
 public:
  explicit BundleAdjustmentNonSvpCameraConstantPoseCostFunctionBackProjection(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Vector2d& point2D)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        observed_x_(point2D(0)),
        observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                     const Eigen::Vector3d& tvec,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraConstantPoseCostFunctionBackProjection<
                CameraNonSvpModel, CameraModel>,
            3, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraConstantPoseCostFunctionBackProjection(
            qvec, tvec, point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  const T* const non_svp_params, T* residuals) const {
    const T qvec[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};
    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += T(tx_);
    point3D_camera[1] += T(ty_);
    point3D_camera[2] += T(tz_);

    // Compute depth of the 3D point to the camera center
    const T depth = ceres::sqrt(point3D_camera[0] * point3D_camera[0] +
                                point3D_camera[1] * point3D_camera[1] +
                                point3D_camera[2] * point3D_camera[2]);

    // compute back-projected 3D point given depth
    Eigen::Matrix<T, 3, 1> back_projection;
    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, T(observed_x_), T(observed_y_), depth,
        &back_projection);

    residuals[0] = back_projection[0] - point3D_camera[0];
    residuals[1] = back_projection[1] - point3D_camera[1];
    residuals[2] = back_projection[2] - point3D_camera[2];

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const double observed_x_;
  const double observed_y_;
};

// Bundle adjustment cost function for Non-Single-View-Point cameras
// using back-projected ray-point distance error.
// Variables include camera calibration, non-SVP camera parameters and point
// parameters
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraConstantPoseCostFunctionPointRayDistance {
 public:
  explicit BundleAdjustmentNonSvpCameraConstantPoseCostFunctionPointRayDistance(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Vector2d& point2D)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        observed_x_(point2D(0)),
        observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                     const Eigen::Vector3d& tvec,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraConstantPoseCostFunctionPointRayDistance<
                CameraNonSvpModel, CameraModel>,
            1, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraConstantPoseCostFunctionPointRayDistance(
            qvec, tvec, point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  const T* const non_svp_params, T* residuals) const {
    const T qvec[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};
    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += T(tx_);
    point3D_camera[1] += T(ty_);
    point3D_camera[2] += T(tz_);

    // compute back-projected 3D ray
    Eigen::Matrix<T, 3, 1> ray_ori = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> ray_dir = Eigen::Matrix<T, 3, 1>::Zero();
    ray_dir(2) = T(1.0);
    CameraNonSvpModel::template ImageToWorld<CameraModel, T>(
        non_svp_params, camera_params, T(observed_x_), T(observed_y_), &ray_ori,
        &ray_dir);

    // compute point-ray distance
    Eigen::Matrix<T, 3, 1> point(point3D_camera[0], point3D_camera[1],
                                 point3D_camera[2]);
    residuals[0] = PointToRayDistance(point, ray_ori, ray_dir);

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const double observed_x_;
  const double observed_y_;
};

// Bundle adjustment cost function for Non-Single-View-Point cameras
// using reprojection error function.
// Variables include camera calibration, non-SVP camera parameters and camera
// pose
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraConstantPoint3DCostFunction {
 public:
  explicit BundleAdjustmentNonSvpCameraConstantPoint3DCostFunction(
      const Eigen::Vector2d& point2D, const Eigen::Vector3d& point3D)
      : observed_x_(point2D(0)),
        observed_y_(point2D(1)),
        point3D_x_(point3D(0)),
        point3D_y_(point3D(1)),
        point3D_z_(point3D(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const Eigen::Vector3d& point3D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraConstantPoint3DCostFunction<
                CameraNonSvpModel, CameraModel>,
            2, 4, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraConstantPoint3DCostFunction(point2D,
                                                                    point3D)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const camera_params, const T* const non_svp_params,
                  T* residuals) const {
    const T point3D[3] = {T(point3D_x_), T(point3D_y_), T(point3D_z_)};

    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += tvec[0];
    point3D_camera[1] += tvec[1];
    point3D_camera[2] += tvec[2];

    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const world_point(point3D_camera);

    // Project to image plane.
    T projection[2];
    CameraNonSvpModel::template WorldToImage<CameraModel, T>(
        non_svp_params, camera_params, world_point, &projection[0],
        &projection[1]);

    residuals[0] = projection[0] - T(observed_x_);
    residuals[1] = projection[1] - T(observed_y_);
    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
  const double point3D_x_;
  const double point3D_y_;
  const double point3D_z_;
};

// Bundle adjustment cost function for Non-Single-View-Point cameras
// using back-projected 3D error function.
// Variables include camera calibration, non-SVP camera parameters and camera
// pose
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionBackProjection {
 public:
  explicit BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionBackProjection(
      const Eigen::Vector2d& point2D, const Eigen::Vector3d& point3D)
      : observed_x_(point2D(0)),
        observed_y_(point2D(1)),
        point3D_x_(point3D(0)),
        point3D_y_(point3D(1)),
        point3D_z_(point3D(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const Eigen::Vector3d& point3D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionBackProjection<
                CameraNonSvpModel, CameraModel>,
            3, 4, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionBackProjection(
            point2D, point3D)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const camera_params, const T* const non_svp_params,
                  T* residuals) const {
    const T point3D[3] = {T(point3D_x_), T(point3D_y_), T(point3D_z_)};

    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += tvec[0];
    point3D_camera[1] += tvec[1];
    point3D_camera[2] += tvec[2];

    // Compute depth of the 3D point to the camera center
    const T depth = ceres::sqrt(point3D_camera[0] * point3D_camera[0] +
                                point3D_camera[1] * point3D_camera[1] +
                                point3D_camera[2] * point3D_camera[2]);

    // Compute back-projected 3D point given depth
    Eigen::Matrix<T, 3, 1> back_projection;
    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, T(observed_x_), T(observed_y_), depth,
        &back_projection);

    residuals[0] = back_projection[0] - point3D_camera[0];
    residuals[1] = back_projection[1] - point3D_camera[1];
    residuals[2] = back_projection[2] - point3D_camera[2];

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
  const double point3D_x_;
  const double point3D_y_;
  const double point3D_z_;
};

// Bundle adjustment cost function for Non-Single-View-Point cameras
// using back-projected point-ray distance error
// Variables include camera calibration, non-SVP camera parameters and camera
// pose
template <typename CameraNonSvpModel, typename CameraModel>
class BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionPointRayDistance {
 public:
  explicit BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionPointRayDistance(
      const Eigen::Vector2d& point2D, const Eigen::Vector3d& point3D)
      : observed_x_(point2D(0)),
        observed_y_(point2D(1)),
        point3D_x_(point3D(0)),
        point3D_y_(point3D(1)),
        point3D_z_(point3D(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const Eigen::Vector3d& point3D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionPointRayDistance<
                CameraNonSvpModel, CameraModel>,
            1, 4, 3, CameraModel::kNumParams, CameraNonSvpModel::kNumParams>(
        new BundleAdjustmentNonSvpCameraConstantPoint3DCostFunctionPointRayDistance(
            point2D, point3D)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const camera_params, const T* const non_svp_params,
                  T* residuals) const {
    const T point3D[3] = {T(point3D_x_), T(point3D_y_), T(point3D_z_)};

    // Rotate and translate
    T point3D_camera[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_camera);
    point3D_camera[0] += tvec[0];
    point3D_camera[1] += tvec[1];
    point3D_camera[2] += tvec[2];

    // Compute back-projected 3D ray
    Eigen::Matrix<T, 3, 1> ray_ori = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> ray_dir = Eigen::Matrix<T, 3, 1>::Zero();
    ray_dir(2) = T(1.0);
    CameraNonSvpModel::template ImageToWorld<CameraModel, T>(
        non_svp_params, camera_params, T(observed_x_), T(observed_y_), &ray_ori,
        &ray_dir);

    // Compute point-ray distance
    const Eigen::Matrix<T, 3, 1> point(point3D_camera[0], point3D_camera[1],
                                       point3D_camera[2]);
    residuals[0] = PointToRayDistance(point, ray_ori, ray_dir);

    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
  const double point3D_x_;
  const double point3D_y_;
  const double point3D_z_;
};

// Cost function for minimizing the absolute difference between the camera
// projection center and the tvec prior.
class ProjectionCenterCostFunction {
 public:
  ProjectionCenterCostFunction(const Eigen::Vector3d& projection_center,
                               const Eigen::Vector3d& weight)
      : projection_center_x_(projection_center(0)),
        projection_center_y_(projection_center(1)),
        projection_center_z_(projection_center(2)),
        weight_x_(weight(0)),
        weight_y_(weight(1)),
        weight_z_(weight(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector3d& projection_center,
                                     const Eigen::Vector3d& weight) {
    return (
        new ceres::AutoDiffCostFunction<ProjectionCenterCostFunction, 3, 4, 3>(
            new ProjectionCenterCostFunction(projection_center, weight)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  T* residuals) const {
    // R, t
    // C = -R.T * t
    const T qvec_c2w[4] = {qvec[0], -qvec[1], -qvec[2], -qvec[3]};
    T tvec_c2w[3];
    ceres::UnitQuaternionRotatePoint(qvec_c2w, tvec, tvec_c2w);
    tvec_c2w[0] = T(-1.0) * tvec_c2w[0];
    tvec_c2w[1] = T(-1.0) * tvec_c2w[1];
    tvec_c2w[2] = T(-1.0) * tvec_c2w[2];

    residuals[0] = T(weight_x_) * (tvec_c2w[0] - T(projection_center_x_));
    residuals[1] = T(weight_y_) * (tvec_c2w[1] - T(projection_center_y_));
    residuals[2] = T(weight_z_) * (tvec_c2w[2] - T(projection_center_z_));
    return true;
  }

 private:
  const double projection_center_x_;
  const double projection_center_y_;
  const double projection_center_z_;
  const double weight_x_;
  const double weight_y_;
  const double weight_z_;
};

// Cost function for the absolute difference of a pose with 6-DoF.
class AbsolutePose6DoFCostFunction {
 public:
  AbsolutePose6DoFCostFunction(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<AbsolutePose6DoFCostFunction, 6, 4,
                                           3>(
        new AbsolutePose6DoFCostFunction(qvec, tvec, sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  T* residuals) const {
    const T qvec_ref[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};
    const T tvec_ref[3] = {T(tx_), T(ty_), T(tz_)};

    // Compute the residuals.
    ComputePose6DoFError(qvec, tvec, qvec_ref, tvec_ref, residuals);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// Cost function for the absolute difference of a pose with 6-DoF.
// This cost function also enables optimizing the relative transform between the
// current estimated pose and the target pose.
class AbsolutePose6DoFWithRelTransformCostFunction {
 public:
  AbsolutePose6DoFWithRelTransformCostFunction(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(
      const Eigen::Vector4d& qvec, const Eigen::Vector3d& tvec,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<
        AbsolutePose6DoFWithRelTransformCostFunction, 6, 4, 3, 4, 3>(
        new AbsolutePose6DoFWithRelTransformCostFunction(qvec, tvec,
                                                         sqrt_information));
  }

  // Note: rel_qvec and rel_tvec satisify the following relation:
  // target_pose = relative_pose * current_pose
  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const rel_qvec, const T* rel_tvec,
                  T* residuals) const {
    const T qvec_ref[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};
    const T tvec_ref[3] = {T(tx_), T(ty_), T(tz_)};

    T qvec_target[4];
    T tvec_target[3];

    ceres::QuaternionProduct(rel_qvec, qvec, qvec_target);
    ceres::UnitQuaternionRotatePoint(rel_qvec, tvec, tvec_target);
    tvec_target[0] += rel_tvec[0];
    tvec_target[1] += rel_tvec[1];
    tvec_target[2] += rel_tvec[2];

    // Compute the residuals between the target pose and the reference pose.
    ComputePose6DoFError(qvec_target, tvec_target, qvec_ref, tvec_ref,
                         residuals);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// Cost function for relative pose with 6-DoF.
//
// @param qvec12, tvec12    Transformation from pose 1 to pose 2
class RelativePose6DoFCostFunction {
 public:
  RelativePose6DoFCostFunction(
      const Eigen::Vector4d& qvec12, const Eigen::Vector3d& tvec12,
      const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : qw_(qvec12(0)),
        qx_(qvec12(1)),
        qy_(qvec12(2)),
        qz_(qvec12(3)),
        tx_(tvec12(0)),
        ty_(tvec12(1)),
        tz_(tvec12(2)),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(
      const Eigen::Vector4d& qvec12, const Eigen::Vector3d& tvec12,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<RelativePose6DoFCostFunction, 6, 4,
                                           3, 4, 3>(
        new RelativePose6DoFCostFunction(qvec12, tvec12, sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const qvec1, const T* const tvec1,
                  const T* const qvec2, const T* const tvec2,
                  T* residuals) const {
    // Compute estimated relative pose from pose 1 to pose 2.
    const T inv_qvec1[4] = {qvec1[0], T(-1.0) * qvec1[1], T(-1.0) * qvec1[2],
                            T(-1.0) * qvec1[3]};
    T qvec12[4];
    ceres::QuaternionProduct(qvec2, inv_qvec1, qvec12);
    T tvec12[3];
    ceres::UnitQuaternionRotatePoint(qvec12, tvec1, tvec12);
    tvec12[0] = tvec2[0] - tvec12[0];
    tvec12[1] = tvec2[1] - tvec12[1];
    tvec12[2] = tvec2[2] - tvec12[2];

    const T qvec12_ref[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};
    const T tvec12_ref[3] = {T(tx_), T(ty_), T(tz_)};

    // Compute the residuals.
    ComputePose6DoFError(qvec12, tvec12, qvec12_ref, tvec12_ref, residuals);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// Cost function for relative pose with 5-DoF using the two-view geometry.
//
// @param qvec12, tvec12    Transformation from pose 1 to pose 2
class RelativePose5DoFCostFunction {
 public:
  RelativePose5DoFCostFunction(
      const Eigen::Vector4d& qvec12, const Eigen::Vector3d& tvec12,
      const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : qw_(qvec12(0)),
        qx_(qvec12(1)),
        qy_(qvec12(2)),
        qz_(qvec12(3)),
        tx_(tvec12(0)),
        ty_(tvec12(1)),
        tz_(tvec12(2)),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(
      const Eigen::Vector4d& qvec12, const Eigen::Vector3d& tvec12,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<RelativePose5DoFCostFunction, 6, 4,
                                           3, 4, 3>(
        new RelativePose5DoFCostFunction(qvec12, tvec12, sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const qvec1, const T* const tvec1,
                  const T* const qvec2, const T* const tvec2,
                  T* residuals) const {
    // Compute estimated relative pose from pose 1 to pose 2.
    const T inv_qvec1[4] = {qvec1[0], T(-1.0) * qvec1[1], T(-1.0) * qvec1[2],
                            T(-1.0) * qvec1[3]};
    T qvec12[4];
    ceres::QuaternionProduct(qvec2, inv_qvec1, qvec12);
    T tvec12[3];
    ceres::UnitQuaternionRotatePoint(qvec12, tvec1, tvec12);
    tvec12[0] = tvec2[0] - tvec12[0];
    tvec12[1] = tvec2[1] - tvec12[1];
    tvec12[2] = tvec2[2] - tvec12[2];

    const T norm = ceres::sqrt(tvec12[0] * tvec12[0] + tvec12[1] * tvec12[1] +
                               tvec12[2] * tvec12[2]);
    tvec12[0] = tvec12[0] / norm;
    tvec12[1] = tvec12[1] / norm;
    tvec12[2] = tvec12[2] / norm;

    const T qvec12_ref[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};
    const T tvec12_ref[3] = {T(tx_), T(ty_), T(tz_)};

    // Compute the residuals.
    ComputePose6DoFError(qvec12, tvec12, qvec12_ref, tvec12_ref, residuals);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const double tx_;
  const double ty_;
  const double tz_;
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// Cost function for relative rotation in 3D.
//
// @param qvec12    Relative rotation from coordinate system 1 to coordinate
// system 2.
class RelativeRotation3DCostFunction {
 public:
  RelativeRotation3DCostFunction(
      const Eigen::Vector4d& qvec12,
      const Eigen::Matrix<double, 3, 3>& sqrt_information)
      : qw_(qvec12(0)),
        qx_(qvec12(1)),
        qy_(qvec12(2)),
        qz_(qvec12(3)),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(
      const Eigen::Vector4d& qvec12,
      const Eigen::Matrix<double, 3, 3>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<RelativeRotation3DCostFunction, 3, 4,
                                           4>(
        new RelativeRotation3DCostFunction(qvec12, sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const qvec1, const T* const qvec2,
                  T* residuals) const {
    // Compute estimated relative rotation from 1 to 2.
    const T inv_qvec1[4] = {qvec1[0], T(-1.0) * qvec1[1], T(-1.0) * qvec1[2],
                            T(-1.0) * qvec1[3]};

    T qvec12[4];
    ceres::QuaternionProduct(qvec2, inv_qvec1, qvec12);

    const T qvec12_ref[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};

    // Compute the residuals.
    ComputeRotation3DError(qvec12, qvec12_ref, residuals);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

 private:
  const double qw_;
  const double qx_;
  const double qy_;
  const double qz_;
  const Eigen::Matrix<double, 3, 3> sqrt_information_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T>
void ComputeRotation3DError(const T* const qvec1, const T* const qvec2,
                            T* error) {
  const T inv_qvec1[4] = {qvec1[0], T(-1.0) * qvec1[1], T(-1.0) * qvec1[2],
                          T(-1.0) * qvec1[3]};

  T qvec12[4];
  ceres::QuaternionProduct(qvec2, inv_qvec1, qvec12);
  error[0] = T(2.0) * qvec12[1];
  error[1] = T(2.0) * qvec12[2];
  error[2] = T(2.0) * qvec12[3];
}

template <typename T>
void ComputePose6DoFError(const T* const qvec1, const T* const tvec1,
                          const T* const qvec2, const T* const tvec2,
                          T error[6]) {
  // Compute relative transformation from 1 to 2.
  const T inv_qvec1[4] = {qvec1[0], T(-1.0) * qvec1[1], T(-1.0) * qvec1[2],
                          T(-1.0) * qvec1[3]};
  T qvec12[4];
  ceres::QuaternionProduct(qvec2, inv_qvec1, qvec12);
  T tvec12[3];
  ceres::UnitQuaternionRotatePoint(qvec12, tvec1, tvec12);
  tvec12[0] = tvec2[0] - tvec12[0];
  tvec12[1] = tvec2[1] - tvec12[1];
  tvec12[2] = tvec2[2] - tvec12[2];

  // [ translation residuals];
  // [ orientation (3x1) reisduals]
  error[0] = tvec12[0];
  error[1] = tvec12[1];
  error[2] = tvec12[2];
  error[3] = T(2.0) * qvec12[1];
  error[4] = T(2.0) * qvec12[2];
  error[5] = T(2.0) * qvec12[3];
}

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_COST_FUNCTIONS_H_
