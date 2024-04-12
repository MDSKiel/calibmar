#pragma once

#include "calibmar/core/pixmap.h"
#include <colmap/scene/camera.h>

namespace calibmar::undistort {

  /// Create an equivalent pinhole camera from provided camera effectively copying size, principal point and focal length
  /// @param camera Distorted camera
  /// @return Pinhole camera
  colmap::Camera CreateUndistortedCamera(colmap::Camera camera);

  /// Undistort an image based on the provided camera. Takes ownership of the input image to avoid a copy (because of the
  /// underlying library).
  /// @param distorted_pixmap input image
  /// @param distorted_camera input distorted camera
  /// @param undistorted_pixmap output image
  /// @param undistorted_camera output undistorted camera
  void UndistortImage(std::unique_ptr<Pixmap> distorted_pixmap, const colmap::Camera& distorted_camera,
                      Pixmap& undistorted_pixmap, colmap::Camera& undistorted_camera);
}