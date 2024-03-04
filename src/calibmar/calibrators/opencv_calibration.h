#pragma once

#include "calibmar/core/camera_models.h"

#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>
#include <colmap/scene/camera.h>

namespace calibmar {
  namespace opencv_calibration {

    // Calibrate camera from sets of 2D-3D correspondences.
    //
    // @param object_points Sets of 3D points.
    // @param image_points Sets of corresponding 2D image points.
    // @param camera Camera that will be calibrated, optionally containing an initial parameter guess.
    // @param use_intrinsic_guess If camera intrinsics should be used as an initial guess.
    // @param fast Use LU instead of SVD decomposition for solving. Faster but potentially less precise (from opencv).
    // @param rotation_vecs Output sets of rotational part of the pose.
    // @param translation_vecs Output sets of corresponding translational part of the pose.
    // @param std_deviations_intrinsics Estimated standard deviation of the camera intrinsics.
    // @param std_deviations_extrinsics Estimated standard deviation of the pose extrinsics.
    // @param per_view_rms Per view RMS.
    // @return Overall RMS
    double CalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Quaterniond*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs, std::vector<double>& std_deviations_intrinsics,
                           std::vector<double>& std_deviations_extrinsics, std::vector<double>& per_view_rms);

    // Overload not calculating std deviations and per view RMS.
    double CalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Quaterniond*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs);

    // Calibrate stereo cameras. 2D point sets are provided per camera and must correspond to the same 3D object points
    //
    // @param object_points Sets of 3D points.
    // @param image_points1 Sets of corresponding 2D image points of camera 1
    // @param image_points2 Sets of corresponding 2D image points of camera 2
    // @param camera1 Camera 1 that will be calibrated, optionally containing an initial parameter guess.
    // @param camera2 Camera 2 that will be calibrated, optionally containing an initial parameter guess.
    // @param pose The transformation which transforms a point from the coordinate system of camera 1 to camera 2.
    // @param use_intrinsic_guess If camera intrinsics should be used as an initial guess.
    // @param fix_intrinsic If only the relative stereo camera pose should be optimized and intrinsics kept fixed.
    // @param same_focal_length If the focal length should be kept the same for both cameras
    // @param per_view_rms The resulting per view RMS. One for each camera
    // @return Overall RMS
    double StereoCalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                                 const std::vector<std::vector<Eigen::Vector2d>>& image_points1,
                                 const std::vector<std::vector<Eigen::Vector2d>>& image_points2, colmap::Camera& camera1,
                                 colmap::Camera& camera2, colmap::Rigid3d& pose, bool use_intrinsic_guess, bool fix_intrinsic,
                                 bool same_focal_length, std::vector<std::vector<double>>& per_view_rms);
  }
}