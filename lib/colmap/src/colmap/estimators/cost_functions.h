// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
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

#pragma once

#include "colmap/geometry/rigid3.h"
#include "colmap/sensor/ray3d.h"
#include "colmap/util/eigen_alignment.h"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace colmap {

template <typename T>
using EigenVector3Map = Eigen::Map<const Eigen::Matrix<T, 3, 1>>;
template <typename T>
using EigenQuaternionMap = Eigen::Map<const Eigen::Quaternion<T>>;

// Standard bundle adjustment cost function for variable
// camera pose, calibration, and point parameters.
template <typename CameraModel>
class ReprojErrorCostFunction {
 public:
  explicit ReprojErrorCostFunction(const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (
        new ceres::AutoDiffCostFunction<ReprojErrorCostFunction<CameraModel>,
                                        2,
                                        4,
                                        3,
                                        3,
                                        CameraModel::num_params>(
            new ReprojErrorCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_rotation,
                  const T* const cam_from_world_translation,
                  const T* const point3D,
                  const T* const camera_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        EigenQuaternionMap<T>(cam_from_world_rotation) *
            EigenVector3Map<T>(point3D) +
        EigenVector3Map<T>(cam_from_world_translation);
    CameraModel::ImgFromCam(camera_params,
                            point3D_in_cam[0],
                            point3D_in_cam[1],
                            point3D_in_cam[2],
                            &residuals[0],
                            &residuals[1]);
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
class ReprojErrorConstantPoseCostFunction {
 public:
  ReprojErrorConstantPoseCostFunction(const Rigid3d& cam_from_world,
                                      const Eigen::Vector2d& point2D)
      : cam_from_world_(cam_from_world),
        observed_x_(point2D(0)),
        observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Rigid3d& cam_from_world,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            ReprojErrorConstantPoseCostFunction<CameraModel>,
            2,
            3,
            CameraModel::num_params>(
        new ReprojErrorConstantPoseCostFunction(cam_from_world, point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D,
                  const T* const camera_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        cam_from_world_.rotation.cast<T>() * EigenVector3Map<T>(point3D) +
        cam_from_world_.translation.cast<T>();
    CameraModel::ImgFromCam(camera_params,
                            point3D_in_cam[0],
                            point3D_in_cam[1],
                            point3D_in_cam[2],
                            &residuals[0],
                            &residuals[1]);
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);
    return true;
  }

 private:
  const Rigid3d& cam_from_world_;
  const double observed_x_;
  const double observed_y_;
};

// Bundle adjustment cost function for variable
// camera pose and calibration parameters, and fixed point.
template <typename CameraModel>
class ReprojErrorConstantPoint3DCostFunction {
 public:
  ReprojErrorConstantPoint3DCostFunction(const Eigen::Vector2d& point2D,
                                         const Eigen::Vector3d& point3D)
      : observed_x_(point2D(0)),
        observed_y_(point2D(1)),
        point3D_x_(point3D(0)),
        point3D_y_(point3D(1)),
        point3D_z_(point3D(2)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const Eigen::Vector3d& point3D) {
    return (new ceres::AutoDiffCostFunction<
            ReprojErrorConstantPoint3DCostFunction<CameraModel>,
            2,
            4,
            3,
            CameraModel::num_params>(
        new ReprojErrorConstantPoint3DCostFunction(point2D, point3D)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_rotation,
                  const T* const cam_from_world_translation,
                  const T* const camera_params,
                  T* residuals) const {
    Eigen::Matrix<T, 3, 1> point3D;
    point3D[0] = T(point3D_x_);
    point3D[1] = T(point3D_y_);
    point3D[2] = T(point3D_z_);

    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        EigenQuaternionMap<T>(cam_from_world_rotation) * point3D +
        EigenVector3Map<T>(cam_from_world_translation);
    CameraModel::ImgFromCam(camera_params,
                            point3D_in_cam[0],
                            point3D_in_cam[1],
                            point3D_in_cam[2],
                            &residuals[0],
                            &residuals[1]);
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
class RigReprojErrorCostFunction {
 public:
  explicit RigReprojErrorCostFunction(const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (
        new ceres::AutoDiffCostFunction<RigReprojErrorCostFunction<CameraModel>,
                                        2,
                                        4,
                                        3,
                                        4,
                                        3,
                                        3,
                                        CameraModel::num_params>(
            new RigReprojErrorCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_rig_rotation,
                  const T* const cam_from_rig_translation,
                  const T* const rig_from_world_rotation,
                  const T* const rig_from_world_translation,
                  const T* const point3D,
                  const T* const camera_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        EigenQuaternionMap<T>(cam_from_rig_rotation) *
            (EigenQuaternionMap<T>(rig_from_world_rotation) *
                 EigenVector3Map<T>(point3D) +
             EigenVector3Map<T>(rig_from_world_translation)) +
        EigenVector3Map<T>(cam_from_rig_translation);
    CameraModel::ImgFromCam(camera_params,
                            point3D_in_cam[0],
                            point3D_in_cam[1],
                            point3D_in_cam[2],
                            &residuals[0],
                            &residuals[1]);
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
// and should be down-projected using `SphereManifold`.
class SampsonErrorCostFunction {
 public:
  SampsonErrorCostFunction(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2)
      : x1_(x1(0)), y1_(x1(1)), x2_(x2(0)), y2_(x2(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& x1,
                                     const Eigen::Vector2d& x2) {
    return (new ceres::AutoDiffCostFunction<SampsonErrorCostFunction, 1, 4, 3>(
        new SampsonErrorCostFunction(x1, x2)));
  }

  template <typename T>
  bool operator()(const T* const cam2_from_cam1_rotation,
                  const T* const cam2_from_cam1_translation,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 3> R =
        EigenQuaternionMap<T>(cam2_from_cam1_rotation).toRotationMatrix();

    // Matrix representation of the cross product t x R.
    Eigen::Matrix<T, 3, 3> t_x;
    t_x << T(0), -cam2_from_cam1_translation[2], cam2_from_cam1_translation[1],
        cam2_from_cam1_translation[2], T(0), -cam2_from_cam1_translation[0],
        -cam2_from_cam1_translation[1], cam2_from_cam1_translation[0], T(0);

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

// Cost function for the absolute difference between the estimated camera
// center and the measured camera center.
class AbsolutePositionErrorCostFunction {
 public:
  AbsolutePositionErrorCostFunction(const Eigen::Vector3d& position_measured,
                                    const Eigen::Matrix3d& sqrt_information)
      : position_measured_(position_measured),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(const Eigen::Vector3d& position_measured,
                                     const Eigen::Matrix3d& sqrt_information) {
    return new ceres::
        AutoDiffCostFunction<AbsolutePositionErrorCostFunction, 3, 4, 3>(
            new AbsolutePositionErrorCostFunction(position_measured,
                                                  sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_rotation,
                  const T* const cam_from_world_translation,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 1> position_estimated =
        EigenQuaternionMap<T>(cam_from_world_rotation).inverse() *
        -EigenVector3Map<T>(cam_from_world_translation);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
    residuals_map = position_estimated - position_measured_.cast<T>();
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.cast<T>());

    return true;
  }

 private:
  const Eigen::Vector3d position_measured_;
  const Eigen::Matrix3d sqrt_information_;
};

// Cost function for the absolute difference in 6-DOFs between the estimated and
// reference pose.
class AbsolutePoseErrorCostFunction {
 public:
  AbsolutePoseErrorCostFunction(const Rigid3d& tform_measured,
                                const Eigen::Matrix6d& sqrt_information)
      : tform_measured_(tform_measured), sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(const Rigid3d& tform_measured,
                                     const Eigen::Matrix6d& sqrt_information) {
    return new ceres::
        AutoDiffCostFunction<AbsolutePoseErrorCostFunction, 6, 4, 3>(
            new AbsolutePoseErrorCostFunction(tform_measured,
                                              sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const rotation_estimated,
                  const T* const translation_estimated,
                  T* residuals) const {
    // Compute the relative transform from the estimated tform to the measured
    // tform.
    const Eigen::Quaternion<T> measured_from_estimated_rotation =
        tform_measured_.rotation.cast<T>() *
        EigenQuaternionMap<T>(rotation_estimated).inverse();
    const Eigen::Matrix<T, 3, 1> measured_from_estimated_translation =
        tform_measured_.translation.cast<T>() -
        measured_from_estimated_rotation *
            EigenVector3Map<T>(translation_estimated);

    // Compute the residuals.
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    residuals_map.template block<3, 1>(0, 0) =
        T(2.0) * measured_from_estimated_rotation.vec();
    residuals_map.template block<3, 1>(3, 0) =
        measured_from_estimated_translation;
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.cast<T>());

    return true;
  }

 private:
  const Rigid3d tform_measured_;
  const Eigen::Matrix6d sqrt_information_;
};

// Cost function for the absolute difference in 6-DOFs between the estimated and
// reference pose. This cost function also enables optimizing the relative
// transform between the current estimated pose and the target pose.
class AbsolutePoseErrorWithRelTformCostFunction {
 public:
  AbsolutePoseErrorWithRelTformCostFunction(
      const Rigid3d& tform_measured, const Eigen::Matrix6d& sqrt_information)
      : tform_measured_(tform_measured), sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(const Rigid3d& tform_measured,
                                     const Eigen::Matrix6d& sqrt_information) {
    return new ceres::AutoDiffCostFunction<
        AbsolutePoseErrorWithRelTformCostFunction,
        6,
        4,
        3,
        4,
        3>(new AbsolutePoseErrorWithRelTformCostFunction(tform_measured,
                                                         sqrt_information));
  }

  // Note: target_from_current_rotation and target_from_current_translation
  // satisify the following relation: target_pose = target_from_current *
  // estimated_pose
  template <typename T>
  bool operator()(const T* const rotation_estimated,
                  const T* const translation_estimated,
                  const T* const target_from_current_rotation,
                  const T* target_from_current_translation,
                  T* residuals) const {
    const Eigen::Quaternion<T> rotation_target_estimated =
        (EigenQuaternionMap<T>(target_from_current_rotation) *
         EigenQuaternionMap<T>(rotation_estimated))
            .normalized();
    const Eigen::Matrix<T, 3, 1> translation_target_estimated =
        EigenVector3Map<T>(target_from_current_translation) +
        (EigenQuaternionMap<T>(target_from_current_rotation) *
         EigenVector3Map<T>(translation_estimated));

    // Compute the relative transform from the estimated tform to the measured
    // tform.
    const Eigen::Quaternion<T> measured_from_estimated_rotation =
        tform_measured_.rotation.cast<T>() *
        rotation_target_estimated.inverse();
    const Eigen::Matrix<T, 3, 1> measured_from_estimated_translation =
        tform_measured_.translation.cast<T>() -
        measured_from_estimated_rotation * translation_target_estimated;

    // Compute the residuals.
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    residuals_map.template block<3, 1>(0, 0) =
        T(2.0) * measured_from_estimated_rotation.vec();
    residuals_map.template block<3, 1>(3, 0) =
        measured_from_estimated_translation;
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.cast<T>());

    return true;
  }

 private:
  const Rigid3d tform_measured_;
  const Eigen::Matrix6d sqrt_information_;
};

// Cost function for relative pose in 6-DOFs. Given the measured relative pose
// from a to b, this cost function optimizes pose a and pose b to minimize the
// relative pose error.
//
// @param qvec12, tvec12    Transformation from pose 1 to pose 2
class RelativePoseError6DoFCostFunction {
 public:
  RelativePoseError6DoFCostFunction(const Rigid3d& cam2_from_cam1_measured,
                                    const Eigen::Matrix6d& sqrt_information)
      : cam2_from_cam1_measured_(cam2_from_cam1_measured),
        sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(const Rigid3d& cam2_from_cam1_measured,
                                     const Eigen::Matrix6d& sqrt_information) {
    return new ceres::
        AutoDiffCostFunction<RelativePoseError6DoFCostFunction, 6, 4, 3, 4, 3>(
            new RelativePoseError6DoFCostFunction(cam2_from_cam1_measured,
                                                  sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const cam1_from_world_rotation,
                  const T* const cam1_from_world_translation,
                  const T* const cam2_from_world_rotation,
                  const T* const cam2_from_world_translation,
                  T* residuals) const {
    // Compute estimated relative transform from a to b.

    const Eigen::Quaternion<T> cam2_from_cam1_rotation =
        EigenQuaternionMap<T>(cam2_from_world_rotation) *
        EigenQuaternionMap<T>(cam1_from_world_rotation).inverse();
    const Eigen::Matrix<T, 3, 1> cam2_from_cam1_translation =
        EigenVector3Map<T>(cam2_from_world_translation) -
        cam2_from_cam1_rotation *
            EigenVector3Map<T>(cam1_from_world_translation);

    const Eigen::Quaternion<T> measured_from_estimated_rotation =
        cam2_from_cam1_measured_.rotation.cast<T>() *
        cam2_from_cam1_rotation.inverse();
    const Eigen::Matrix<T, 3, 1> measured_from_estimated_translation =
        cam2_from_cam1_measured_.translation.cast<T>() -
        measured_from_estimated_rotation * cam2_from_cam1_translation;

    // Compute the residuals.
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    residuals_map.template block<3, 1>(0, 0) =
        T(2.0) * measured_from_estimated_rotation.vec();
    residuals_map.template block<3, 1>(3, 0) =
        measured_from_estimated_translation;
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.cast<T>());

    return true;
  }

 private:
  const Rigid3d cam2_from_cam1_measured_;
  const Eigen::Matrix6d sqrt_information_;
};

// Cost function for smooth motion constraint.
class SmoothMotionCostFunction {
 public:
  SmoothMotionCostFunction(const Eigen::Matrix6d& sqrt_information)
      : sqrt_information_(sqrt_information) {}

  static ceres::CostFunction* Create(const Eigen::Matrix6d& sqrt_information) {
    return new ceres::
        AutoDiffCostFunction<SmoothMotionCostFunction, 6, 4, 3, 4, 3, 4, 3>(
            new SmoothMotionCostFunction(sqrt_information));
  }

  template <typename T>
  bool operator()(const T* const cam1_from_world_rotation,
                  const T* const cam1_from_world_translation,
                  const T* const cam2_from_world_rotation,
                  const T* const cam2_from_world_translation,
                  const T* const cam3_from_world_rotation,
                  const T* const cam3_from_world_translation,
                  T* residuals) const {
    // Compute relative transform from 1 to 2.
    const Eigen::Quaternion<T> cam2_from_cam1_rotation =
        EigenQuaternionMap<T>(cam2_from_world_rotation) *
        EigenQuaternionMap<T>(cam1_from_world_rotation).inverse();
    const Eigen::Matrix<T, 3, 1> cam2_from_cam1_translation =
        EigenVector3Map<T>(cam2_from_world_translation) -
        cam2_from_cam1_rotation *
            EigenVector3Map<T>(cam1_from_world_translation);
    // Compute relative transfortransformmation from 2 to 3.
    const Eigen::Quaternion<T> cam3_from_cam2_rotation =
        EigenQuaternionMap<T>(cam3_from_world_rotation) *
        EigenQuaternionMap<T>(cam2_from_world_rotation).inverse();
    const Eigen::Matrix<T, 3, 1> cam3_from_cam2_translation =
        EigenVector3Map<T>(cam3_from_world_translation) -
        cam3_from_cam2_rotation *
            EigenVector3Map<T>(cam2_from_world_translation);

    // Compute residuals such that the relative transform from 1 to 2
    // should be similar to relative transform from 2 to 3.
    const Eigen::Quaternion<T> diff_rotation =
        cam3_from_cam2_rotation * cam2_from_cam1_rotation.inverse();
    const Eigen::Matrix<T, 3, 1> diff_translation =
        cam3_from_cam2_translation - diff_rotation * cam2_from_cam1_translation;

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals_map(residuals);
    residuals_map.template block<3, 1>(0, 0) = T(2.0) * diff_rotation.vec();
    residuals_map.template block<3, 1>(3, 0) = diff_translation;
    // Scale the residuals by the measurement uncertainty.
    residuals_map.applyOnTheLeft(sqrt_information_.cast<T>());

    return true;
  }

 private:
  const Eigen::Matrix6d sqrt_information_;
};

// Refractive bundle adjustment cost function for variable
// camera pose, calibration, and point parameters.
template <typename CameraRefracModel, typename CameraModel>
class ReprojErrorRefracCostFunction {
 public:
  explicit ReprojErrorRefracCostFunction(const Eigen::Vector2d& point2D)
      : observed_x_(point2D(0)), observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            ReprojErrorRefracCostFunction<CameraRefracModel, CameraModel>,
            2,
            4,
            3,
            3,
            CameraModel::num_params,
            CameraRefracModel::num_params>(
        new ReprojErrorRefracCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_rotation,
                  const T* const cam_from_world_translation,
                  const T* const point3D,
                  const T* const camera_params,
                  const T* const refrac_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        EigenQuaternionMap<T>(cam_from_world_rotation) *
            EigenVector3Map<T>(point3D) +
        EigenVector3Map<T>(cam_from_world_translation);

    // Compute the refracted ray.
    Eigen::Matrix<T, 3, 1> ray_ori;
    Eigen::Matrix<T, 3, 1> ray_dir;
    CameraRefracModel::template CamFromImg<CameraModel, T>(camera_params,
                                                           refrac_params,
                                                           T(observed_x_),
                                                           T(observed_y_),
                                                           &ray_ori,
                                                           &ray_dir);

    // Compute the virtual from real transformation.
    Eigen::Matrix<T, 3, 1> refrac_axis;
    CameraRefracModel::RefractionAxis(refrac_params, &refrac_axis);
    const Eigen::Quaternion<T> virtual_from_real_rotation(
        T(1), T(0), T(0), T(0));

    Eigen::Matrix<T, 3, 1> virtual_cam_center;
    IntersectLinesWithTolerance<T>(Eigen::Matrix<T, 3, 1>::Zero(),
                                   -refrac_axis,
                                   ray_ori,
                                   -ray_dir,
                                   virtual_cam_center);

    const Eigen::Matrix<T, 3, 1> virtual_from_real_translation =
        virtual_from_real_rotation * -virtual_cam_center;

    const Eigen::Matrix<T, 2, 1> cam_point = ray_dir.hnormalized();

    const T f = camera_params[0];
    const T c1 = T(observed_x_) - f * cam_point[0];
    const T c2 = T(observed_y_) - f * cam_point[1];

    // Finally, do simple pinhole projection.
    const Eigen::Matrix<T, 3, 1> point3D_in_virtual =
        virtual_from_real_rotation * point3D_in_cam +
        virtual_from_real_translation;

    residuals[0] = f * point3D_in_virtual[0] / point3D_in_virtual[2] + c1;
    residuals[1] = f * point3D_in_virtual[1] / point3D_in_virtual[2] + c2;
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);
    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
};

// Refractive Bundle adjustment cost function for variable
// camera calibration and point parameters, and fixed camera pose.
template <typename CameraRefracModel, typename CameraModel>
class ReprojErrorRefracConstantPoseCostFunction {
 public:
  ReprojErrorRefracConstantPoseCostFunction(const Rigid3d& cam_from_world,
                                            const Eigen::Vector2d& point2D)
      : cam_from_world_(cam_from_world),
        observed_x_(point2D(0)),
        observed_y_(point2D(1)) {}

  static ceres::CostFunction* Create(const Rigid3d& cam_from_world,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            ReprojErrorRefracConstantPoseCostFunction<CameraRefracModel,
                                                      CameraModel>,
            2,
            3,
            CameraModel::num_params,
            CameraRefracModel::num_params>(
        new ReprojErrorRefracConstantPoseCostFunction(cam_from_world,
                                                      point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D,
                  const T* const camera_params,
                  const T* const refrac_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 3, 1> point3D_in_cam =
        cam_from_world_.rotation.cast<T>() * EigenVector3Map<T>(point3D) +
        cam_from_world_.translation.cast<T>();

    // Compute the refracted ray.
    Eigen::Matrix<T, 3, 1> ray_ori;
    Eigen::Matrix<T, 3, 1> ray_dir;
    CameraRefracModel::template CamFromImg<CameraModel, T>(camera_params,
                                                           refrac_params,
                                                           T(observed_x_),
                                                           T(observed_y_),
                                                           &ray_ori,
                                                           &ray_dir);

    // Compute the virtual from real transformation.
    Eigen::Matrix<T, 3, 1> refrac_axis;
    CameraRefracModel::RefractionAxis(refrac_params, &refrac_axis);
    Eigen::Quaternion<T> virtual_from_real_rotation(T(1), T(0), T(0), T(0));

    Eigen::Matrix<T, 3, 1> virtual_cam_center;
    IntersectLinesWithTolerance<T>(Eigen::Matrix<T, 3, 1>::Zero(),
                                   -refrac_axis,
                                   ray_ori,
                                   -ray_dir,
                                   virtual_cam_center);

    const Eigen::Matrix<T, 3, 1> virtual_from_real_translation =
        virtual_from_real_rotation * -virtual_cam_center;

    const Eigen::Matrix<T, 2, 1> cam_point = ray_dir.hnormalized();

    const T f = camera_params[0];
    const T c1 = T(observed_x_) - f * cam_point[0];
    const T c2 = T(observed_y_) - f * cam_point[1];

    // Finally, do simple pinhole projection.
    const Eigen::Matrix<T, 3, 1> point3D_in_virtual =
        virtual_from_real_rotation * point3D_in_cam +
        virtual_from_real_translation;

    residuals[0] = f * point3D_in_virtual[0] / point3D_in_virtual[2] + c1;
    residuals[1] = f * point3D_in_virtual[1] / point3D_in_virtual[2] + c2;
    residuals[0] -= T(observed_x_);
    residuals[1] -= T(observed_y_);
    return true;
  }

 private:
  const Rigid3d& cam_from_world_;
  const double observed_x_;
  const double observed_y_;
};

// Cost function for refining generalized relative pose based on the
// Sampson-Error.
//
// First pose is assumed to be located at the origin with 0 rotation. Second
// pose is assumed to be on the unit sphere around the first pose, i.e. the
// pose of the second camera is parameterized by a 3D rotation and a
// 3D translation with unit norm. `tvec` is therefore over-parameterized as is
// and should be down-projected using `SphereManifold`.
class GeneralizedSampsonErrorCostFunction {
 public:
  GeneralizedSampsonErrorCostFunction(const Eigen::Vector2d& x1,
                                      const Eigen::Vector2d& x2,
                                      const Rigid3d& cam1_from_rig1,
                                      const Rigid3d& cam2_from_rig2)
      : x1_(x1(0)),
        y1_(x1(1)),
        x2_(x2(0)),
        y2_(x2(1)),
        cam1_from_rig1_(cam1_from_rig1),
        cam2_from_rig2_(cam2_from_rig2) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& x1,
                                     const Eigen::Vector2d& x2,
                                     const Rigid3d& cam1_from_rig1,
                                     const Rigid3d& cam2_from_rig2) {
    return (
        new ceres::
            AutoDiffCostFunction<GeneralizedSampsonErrorCostFunction, 1, 4, 3>(
                new GeneralizedSampsonErrorCostFunction(
                    x1, x2, cam1_from_rig1, cam2_from_rig2)));
  }

  template <typename T>
  bool operator()(const T* const rig2_from_rig1_rotation,
                  const T* const rig2_from_rig1_translation,
                  T* residuals) const {
    // Compute cam2_from_cam1 transformation.
    const Eigen::Quaternion<T> rig1_from_cam1_rotation =
        cam1_from_rig1_.rotation.cast<T>().inverse();
    const Eigen::Matrix<T, 3, 1> rig1_from_cam1_translation =
        rig1_from_cam1_rotation * -cam1_from_rig1_.translation.cast<T>();

    const Eigen::Quaternion<T> rig2_from_cam1_rotation =
        EigenQuaternionMap<T>(rig2_from_rig1_rotation) *
        rig1_from_cam1_rotation;
    const Eigen::Matrix<T, 3, 1> rig2_from_cam1_translation =
        EigenVector3Map<T>(rig2_from_rig1_translation) +
        EigenQuaternionMap<T>(rig2_from_rig1_rotation) *
            rig1_from_cam1_translation;

    const Eigen::Quaternion<T> cam2_from_cam1_rotation =
        cam2_from_rig2_.rotation.cast<T>() * rig2_from_cam1_rotation;

    const Eigen::Matrix<T, 3, 1> cam2_from_cam1_translation =
        (cam2_from_rig2_.translation.cast<T>() +
         cam2_from_rig2_.rotation.cast<T>() * rig2_from_cam1_translation)
            .normalized();

    const Eigen::Matrix<T, 3, 3> R = cam2_from_cam1_rotation.toRotationMatrix();

    // Matrix representation of the cross product t x R.
    Eigen::Matrix<T, 3, 3> t_x;
    t_x << T(0), -cam2_from_cam1_translation[2], cam2_from_cam1_translation[1],
        cam2_from_cam1_translation[2], T(0), -cam2_from_cam1_translation[0],
        -cam2_from_cam1_translation[1], cam2_from_cam1_translation[0], T(0);

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
  const Rigid3d& cam1_from_rig1_;
  const Rigid3d& cam2_from_rig2_;
};

inline void SetQuaternionManifold(ceres::Problem* problem, double* quat_xyzw) {
#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
  problem->SetManifold(quat_xyzw, new ceres::EigenQuaternionManifold);
#else
  problem->SetParameterization(quat_xyzw,
                               new ceres::EigenQuaternionParameterization);
#endif
}

inline void SetSubsetManifold(int size,
                              const std::vector<int>& constant_params,
                              ceres::Problem* problem,
                              double* params) {
#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
  problem->SetManifold(params,
                       new ceres::SubsetManifold(size, constant_params));
#else
  problem->SetParameterization(
      params, new ceres::SubsetParameterization(size, constant_params));
#endif
}

template <int size>
inline void SetSphereManifold(ceres::Problem* problem, double* params) {
#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
  problem->SetManifold(params, new ceres::SphereManifold<size>);
#else
  problem->SetParameterization(
      params, new ceres::HomogeneousVectorParameterization(size));
#endif
}

}  // namespace colmap
