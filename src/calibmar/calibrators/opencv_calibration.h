#pragma once

#include "calibmar/core/camera_models.h"

#include <Eigen/Core>
#include <colmap/src/base/camera.h>

namespace calibmar {
  namespace opencv_calibration {

    // Calibrate camera from sets of 2D-3D correspondences.
    //
    // @param object_points Sets of 3D points.
    // @param image_points Sets of corresponding 2D image points.
    // @param camera Camera that will be calibrated, optionally containing an initial parameter guess.
    // @param use_intrinsic_guess If camera intrinsics should be used as an initial guess.
    // @param fast Use LU instead of SVD decomposition for solving. Faster but potentially less precise (from opencv).
    // @param rotation_vecs Output sets of rotational part of the pose (as quaternion w, x, y, z).
    // @param translation_vecs Output sets of corresponding translational part of the pose.
    // @param std_deviations_intrinsics Estimated standard deviation of the camera intrinsics.
    // @param std_deviations_extrinsics Estimated standard deviation of the pose extrinsics.
    // @param per_view_rms Per view RMS.
    // @return Overall RMS
    double CalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Vector4d*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs, std::vector<double>& std_deviations_intrinsics,
                           std::vector<double>& std_deviations_extrinsics, std::vector<double>& per_view_rms);

    // Overload not calculating std deviations and per view RMS.
    double CalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Vector4d*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs);
  }
}