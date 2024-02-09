#pragma once

#include "calibmar/core/calibration.h"

#include <colmap/scene/camera.h>
#include <optional>

namespace calibmar {
  namespace non_svp_calibration {

    // Estimate the initial dome offset using the "epipolar like geometry contraint" from
    // "Refractive geometry for underwater domes" (https://doi.org/10.1016/j.isprsjprs.2021.11.006).
    // The camera is expected to be a spherical refractive interface non svp camera and the estimated
    // offset will be added to the camera in place
    //
    // @param points_3D Real world chessboard corner coordinates. Must match shape of points_2D.
    // @param points_2D Detected image chessboard corners.
    // @param pattern_cols_rows Width/height of the points_2D chessboard corners.
    // @param spherical Refractive interface non svp camera, edited in place.
    // @param max_expected_offset_percent The estimated decentering will be set to offset_direction * radius *
    // max_expected_offset_percent. Defaults to 0.2.
    void EstimateInitialDomeOffset(const std::vector<Eigen::Vector3d>& points_3D, const std::vector<Eigen::Vector2d>& points_2D,
                                   const std::pair<int, int> pattern_cols_rows, colmap::Camera& camera,
                                   double max_expected_offset_percent = 0.2);

    // Estimate absolute pose from 2D-3D correspondences for refractive cameras.
    //
    // @param points_2D               Corresponding 2D points.
    // @param points_3D               Corresponding 3D points.
    // @param rotation_quaternion     Estimated rotation component.
    // @param translation             Estimated translation component.
    // @param camera                  Camera for which to estimate pose.
    // @param use_initial_pose_guess  If rotation_quaternion and translation already contain a valid first estimation.
    //
    // @return                        Whether pose is estimated successfully.
    bool EstimateAbsolutePoseRefractiveCamera(const std::vector<Eigen::Vector3d>& points_3D,
                                              const std::vector<Eigen::Vector2d>& points_2D,
                                              Eigen::Quaterniond* rotation_quaternion, Eigen::Vector3d* translation,
                                              colmap::Camera* camera, bool use_initial_pose_guess = false);
}