#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"

#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>
#include <colmap/scene/camera.h>

#include <optional>

namespace calibmar::general_calibration {

  // Undistort image pixel coordinates to image pixel coordinates using the distortion parameters of provided camera. If the
  // camera has no distortion (IsUndistorted() == true), image_points will be copied to undistorted_points.
  // @param image_points Original image pixel locations
  // @param camera Camera belonging to the distorted pixel locations
  // @param undistorted_points Result of undistorting image_points into new pixel coordinates
  void UndistortImagePoints(const std::vector<std::vector<Eigen::Vector2d>>& image_points, const colmap::Camera& camera,
                            std::vector<std::vector<Eigen::Vector2d>>& undistorted_points);

  // Estimate camera intrinsics from observations of a planar calibration target . Uses the method from Z. Zhang, "A flexible new
  // technique for camera calibration", i.e. constraints from homography. Also adds constraints for square pixels (aspect ratio 1)
  // and no skew.
  // @param object_plane_points Points on the planar calibration target in 2D real world coordinates
  // @param image_points Observations of the object points in image pixel coordinates. Must correspond to and exactly match the
  // object_plane_points shape.
  // @param fx Estimated focal length fx
  // @param fy Estimated focal length fy
  // @param cx Estimated x component of the principal point
  // @param cy Estimated y component of the principal point
  void EstimateKFromHomographies(const std::vector<std::vector<Eigen::Vector2d>>& object_plane_points,
                                 const std::vector<std::vector<Eigen::Vector2d>>& image_points, double& fx, double& fy,
                                 double& cx, double& cy);

  // Estimate extrinsic pose given a homography between an object and image plane and the corresponding camera K matrix.
  // @param H Homography between object plane and image plane
  // @param K Camera matrix of the camera (to be removed from H)
  // @param pose Estmated camera pose
  void EstimatePoseFromHomography(Eigen::Matrix3d H, const Eigen::Matrix3d& K, colmap::Rigid3d& pose);

  // Calibrate Camera given planar object points and the corresponding image observations.
  // @param object_points Collection of planar object points (Will typically be the same set of point coordinates multiple times)
  // in world (calibration target) coordinates
  // @param image_points 2D image observations. Must correspond to and exactly match the object points shape.
  // @param camera The camera to be calibrated in place. Width, height and model must be set.
  // @param use_intrinsic_guess If the camera contains valid parameters which should be used for initializing the calibration
  // optimization. (Otherwise the intrinsics will be estimated from homographies).
  // @param poses The resulting extrinsic poses per view.
  void CalibrateCamera(std::vector<std::vector<Eigen::Vector3d>>& object_points,
                       const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                       bool use_intrinsic_guess, std::vector<colmap::Rigid3d>& poses,
                       std::vector<double>* std_deviations_intrinsics = nullptr);

  // Calculate the per view and overall RMS given a set of 2D-3D correspondences, poses and a calibrated camera.
  // @param object_points 3D object points
  // @param image_points 2D image observations
  // @param poses Corresponding camera poses
  // @param camera Calibrated camera
  // @param per_view_rms The per view errors
  // @return The overall RMS calculated as sum(per_view_rms) / per_view_rms.size()
  double CalculateOverallRMS(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                             const std::vector<std::vector<Eigen::Vector2d>>& image_points,
                             const std::vector<colmap::Rigid3d>& poses, const colmap::Camera& camera,
                             std::vector<double>& per_view_rms);

  // Calculate the per view squared error given a set of 2D-3D correspondences, poses and a calibrated camera.
  // @param object_points 3D object points
  // @param image_points 2D image observations
  // @param poses Corresponding camera poses
  // @param camera Calibrated camera
  // @param per_view_se The per view errors
  void CalculateperViewSquaredError(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                                    const std::vector<std::vector<Eigen::Vector2d>>& image_points,
                                    const std::vector<colmap::Rigid3d>& poses, const colmap::Camera& camera,
                                    std::vector<double>& per_view_se);

  // Optimize camera parameters given a set of 2D - 3D correspondences and poses
  // Will optimize camera intrinsics if camera is non refractive and the housing parameters (keeping intrinsics constant) if
  // camera is refractive.
  // @param object_points 3D point sets
  // @param image_points Corresponding 2D image observations
  // @param camera Camera to be optimized
  // @param poses Corresponding poses per views
  // @param std_deviations_intrinsics Optionally giving the estimated standard deviations on either camera or housing parameters
  // depending on camera type
  void OptimizeCamera(std::vector<std::vector<Eigen::Vector3d>>& object_points,
                      const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                      std::vector<colmap::Rigid3d>& poses, std::vector<double>* std_deviations_intrinsics);

  // Overload extracting the required information from a prepared Calibration
  // @param fix_3D_points If 3D points should be considered constant (defaults to true), otherwise a complete bundle adjustment
  // will be performed (keeping the first pose constant)
  void OptimizeCamera(Calibration& calibration, bool fix_3D_points = true);
}