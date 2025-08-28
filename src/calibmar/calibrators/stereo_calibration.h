#pragma once

#include "calibmar/core/camera_models.h"

#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>
#include <colmap/scene/camera.h>

namespace calibmar::stereo_calibration {
  struct StereoStdDeviations {
    std::vector<double>* std_deviations_intrinsics1 = nullptr;
    std::vector<double>* std_deviations_intrinsics2 = nullptr;
    std::vector<double>* std_deviations_extrinsics1 = nullptr;
    std::vector<double>* std_deviations_extrinsics2 = nullptr;
  };

  // Stereo calibrate two cameras from corresponding planar calibration pattern views of each camera. Allows calibration of each
  // camera by itself and then joint optimization of absolute and relative pose aswell as intrinsics.
  // @param object_points1 Collection of planar object points in world (calibration target) coordinates, corresponding to
  // image_points1
  // @param object_points2 Collection of planar object points in world (calibration target) coordinates, corresponding to
  // image_points2
  // @param image_points1 2D image observations of the first camera. Must correspond to and exactly match the object points1 shape
  // @param image_points2 2D image observations of the second camera. Must correspond to and exactly match the object points2
  // shape
  // @param camera1 First camera, considered to be the main camera, i.e. origin of the stereo system
  // @param camera2 Second camera, considered to be the secondary camera for which the pose is estimated relative to the main
  // camera
  // @param use_intrinsic_guess If the camera intrinsics should be used as an initialization for optimization
  // @param fix_intrinsics If the camera intrinsics should be considered known and subsequently kept fixed during optimization.
  // This will result in only estimating the absolute and relative poses.
  // @param relative_pose The transformation which transforms a point from the coordinate system of camera 1 to camera 2.
  // @param poses The absolute poses corresponding to each view (i.e. the poses of the first camera)
  // @param std_deviations If requested, contains the estimated standard deviations of each estimated parameter
  void CalibrateStereoCameras(std::vector<std::vector<Eigen::Vector3d>>& object_points1,
                              std::vector<std::vector<Eigen::Vector3d>>& object_points2,
                              const std::vector<std::vector<Eigen::Vector2d>>& image_points1,
                              const std::vector<std::vector<Eigen::Vector2d>>& image_points2, colmap::Camera& camera1,
                              colmap::Camera& camera2, bool use_intrinsic_guess, bool fix_intrinsics,
                              colmap::Rigid3d& relative_pose, std::vector<colmap::Rigid3d>& poses,
                              StereoStdDeviations* std_deviations = nullptr);
}