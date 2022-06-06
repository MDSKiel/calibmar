#pragma once

#include "calibmar/core/calibration.h"

#include <colmap/src/base/camera.h>
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

    // Estimate absolute pose from 2D-3D correspondences for none svp cameras.
    //
    // @param points_2D               Corresponding 2D points.
    // @param points_3D               Corresponding 3D points.
    // @param rotation_quaternion     Estimated rotation component as unit Quaternion coefficients (w, x, y, z).
    // @param translation             Estimated translation component.
    // @param camera                  Camera for which to estimate pose.
    // @param use_initial_pose_guess  If rotation_quaternion and translation already contain a valid first estimation.
    //
    // @return                        Whether pose is estimated successfully.
    bool EstimateAbsolutePoseNonSvpCamera(const std::vector<Eigen::Vector3d>& points_3D,
                                          const std::vector<Eigen::Vector2d>& points_2D, Eigen::Vector4d* rotation_quaternion,
                                          Eigen::Vector3d* translation, colmap::Camera* camera,
                                          bool use_initial_pose_guess = false);

    // Optimize the non svp (that is: the housing related) parameters of the camera.
    // Expects the calibration to have a camera with a valid first estimation of parameters (housing and intrinsics),
    // as well as a set of images with estimated poses.
    //
    // @param calibration         Calibration that is already parametrized with a initialized camera and pose estimated images.
    //                            Is edited in place.
    // @param housing_params_std  Estimated standard deviations of the housing parameters.
    void OptimizeNonSvpCamera(calibmar::Calibration& calibration, std::vector<double>& housing_params_std);

    // Calculate the mean squared reprojection error for each image in the calibration using the
    // calibration camera.
    //
    // @param calibration   Calibration containing images with estimated poses and a parameterized camera.
    // @param per_view_mse  Output of mean squared error per image.
    void CalculatePerImageMSE(const Calibration& calibration, std::vector<double>& per_image_mse);

    // Calculate the mean squared reprojection error for each image in the calibration using the
    // calibration camera.
    //
    // @param calibration   Calibration containing images with estimated poses and a parameterized camera.
    // @return              Overall mean squared reprojection error.
    double CalculateOverallMSE(const Calibration& calibration);
  }
}