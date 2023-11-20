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

#include "colmap/sensor/ray3d.h"
#include "colmap/util/types.h"

#include <vector>

namespace colmap {

class Rigid3d;

// Camera class that holds the intrinsic parameters. Cameras may be shared
// between multiple images, e.g., if the same "physical" camera took multiple
// pictures with the exact same lens and intrinsics (focal length, etc.).
// This class has a specific distortion model defined by a camera model class.
class Camera {
 public:
  Camera();

  // Access the unique identifier of the camera.
  inline camera_t CameraId() const;
  inline void SetCameraId(camera_t camera_id);

  // Access the camera model.
  inline int ModelId() const;
  std::string ModelName() const;
  void SetModelId(int model_id);
  void SetModelIdFromName(const std::string& model_name);

  // Access the refractive camera model.
  inline int RefracModelId() const;
  std::string RefracModelName() const;
  void SetRefracModelId(int refrac_model_id);
  void SetRefracModelIdFromName(const std::string& refrac_model_name);

  // Access dimensions of the camera sensor.
  inline size_t Width() const;
  inline size_t Height() const;
  inline void SetWidth(size_t width);
  inline void SetHeight(size_t height);

  // Access focal length parameters.
  double MeanFocalLength() const;
  double FocalLength() const;
  double FocalLengthX() const;
  double FocalLengthY() const;
  void SetFocalLength(double focal_length);
  void SetFocalLengthX(double focal_length_x);
  void SetFocalLengthY(double focal_length_y);

  // Check if camera has prior focal length.
  inline bool HasPriorFocalLength() const;
  inline void SetPriorFocalLength(bool prior);

  // Access principal point parameters. Only works if there are two
  // principal point parameters.
  double PrincipalPointX() const;
  double PrincipalPointY() const;
  void SetPrincipalPointX(double ppx);
  void SetPrincipalPointY(double ppy);

  // Get the indices of the parameter groups in the parameter vector.
  const std::vector<size_t>& FocalLengthIdxs() const;
  const std::vector<size_t>& PrincipalPointIdxs() const;
  const std::vector<size_t>& ExtraParamsIdxs() const;

  // Get intrinsic calibration matrix composed from focal length and principal
  // point parameters, excluding distortion parameters.
  Eigen::Matrix3d CalibrationMatrix() const;

  // Get human-readable information about the parameter vector ordering.
  std::string ParamsInfo() const;
  std::string RefracParamsInfo() const;

  // Access the raw parameter vector.
  inline size_t NumParams() const;
  inline const std::vector<double>& Params() const;
  inline std::vector<double>& Params();
  inline double Params(size_t idx) const;
  inline double& Params(size_t idx);
  inline const double* ParamsData() const;
  inline double* ParamsData();
  inline void SetParams(const std::vector<double>& params);

  // Access the raw refractive parameter vector.
  inline size_t NumRefracParams() const;
  inline const std::vector<double>& RefracParams() const;
  inline std::vector<double>& RefracParams();
  inline double RefracParams(const size_t idx) const;
  inline double& RefracParams(const size_t idx);
  inline const double* RefracParamsData() const;
  inline double* RefracParamsData();
  inline void SetRefracParams(const std::vector<double>& refrac_params);

  // Concatenate parameters as comma-separated list.
  std::string ParamsToString() const;
  std::string RefracParamsToString() const;

  // Set camera parameters from comma-separated list.
  bool SetParamsFromString(const std::string& string);

  // Set refractive parameters from comma-separated list.
  bool SetRefracParamsFromString(const std::string& string);

  // Check whether parameters are valid, i.e. the parameter vector has
  // the correct dimensions that match the specified camera model.
  bool VerifyParams() const;

  // Check whether refractive parameters are valid, i.e. the parameter vector
  // has the correct dimensions that match the specified refractive camera
  // model.
  bool VerifyRefracParams() const;

  // Check whether camera is already undistorted
  bool IsUndistorted() const;

  // Check whether the camera is a refractive camera.
  bool IsCameraRefractive() const;

  // Check whether camera has bogus parameters.
  bool HasBogusParams(double min_focal_length_ratio,
                      double max_focal_length_ratio,
                      double max_extra_param) const;

  // Initialize parameters for given camera model and focal length, and set
  // the principal point to be the image center.
  void InitializeWithId(int model_id,
                        double focal_length,
                        size_t width,
                        size_t height);
  void InitializeWithName(const std::string& model_name,
                          double focal_length,
                          size_t width,
                          size_t height);

  // Project point in image plane to world / infinity.
  Eigen::Vector2d CamFromImg(const Eigen::Vector2d& image_point) const;

  // Convert pixel threshold in image plane to camera frame.
  double CamFromImgThreshold(double threshold) const;

  // Project point from camera frame to image plane.
  Eigen::Vector2d ImgFromCam(const Eigen::Vector2d& cam_point) const;

  // Project point in image plane to world as 3D ray using refractive camera
  // model.
  Ray3D CamFromImgRefrac(const Eigen::Vector2d& image_point) const;

  // Project point in image plane to world given a depth.
  Eigen::Vector3d CamFromImgRefracPoint(const Eigen::Vector2d& image_point,
                                        double depth) const;

  // Project point from camera frame to image plane using refractive camera
  // model.
  Eigen::Vector2d ImgFromCamRefrac(const Eigen::Vector3d& cam_point) const;

  // Rescale camera dimensions and accordingly the focal length and
  // and the principal point.
  void Rescale(double scale);
  void Rescale(size_t width, size_t height);

  // Get the indices of the parameter groups in the parameter vector.
  const std::vector<size_t>& OptimizableRefracParamsIdxs() const;

  // Return the refraction axis if the camera is refractive.
  Eigen::Vector3d RefractionAxis() const;

  Eigen::Vector3d VirtualCameraCenter(const Ray3D& ray_refrac) const;

  // If the camera is refractive, this function returns a simple pinhole virtual
  // camera which observes the `cam_point` from the `image_point` perspectively.
  Camera VirtualCamera(const Eigen::Vector2d& image_point,
                       const Eigen::Vector2d& cam_point) const;

  void ComputeVirtual(const Eigen::Vector2d& point2D,
                      Camera& virtual_camera,
                      Rigid3d& virtual_from_real) const;

  void ComputeVirtuals(const std::vector<Eigen::Vector2d>& points2D,
                       std::vector<Camera>& virtual_cameras,
                       std::vector<Rigid3d>& virtual_from_reals) const;

 private:
  // The unique identifier of the camera. If the identifier is not specified
  // it is set to `kInvalidCameraId`.
  camera_t camera_id_;

  // The identifier of the camera model. If the camera model is not specified
  // the identifier is `kInvalidCameraModelId`.
  int model_id_;

  // The dimensions of the image, 0 if not initialized.
  size_t width_;
  size_t height_;

  // The focal length, principal point, and extra parameters. If the camera
  // model is not specified, this vector is empty.
  std::vector<double> params_;

  // Whether there is a safe prior for the focal length,
  // e.g. manually provided or extracted from EXIF
  bool prior_focal_length_;

  // The identifier of the refractive camera model. If the camera refractive
  // model is not specified or the camera is not refractive, the identifier is
  // `kInvalidCameraRefracModelId`
  int refrac_model_id_;

  // Refractive camera model parameters. If the camera refractive model is not
  // specified, this vector is empty.
  std::vector<double> refrac_params_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

camera_t Camera::CameraId() const { return camera_id_; }

void Camera::SetCameraId(const camera_t camera_id) { camera_id_ = camera_id; }

int Camera::ModelId() const { return model_id_; }

int Camera::RefracModelId() const { return refrac_model_id_; }

size_t Camera::Width() const { return width_; }

size_t Camera::Height() const { return height_; }

void Camera::SetWidth(const size_t width) { width_ = width; }

void Camera::SetHeight(const size_t height) { height_ = height; }

bool Camera::HasPriorFocalLength() const { return prior_focal_length_; }

void Camera::SetPriorFocalLength(const bool prior) {
  prior_focal_length_ = prior;
}

size_t Camera::NumParams() const { return params_.size(); }

const std::vector<double>& Camera::Params() const { return params_; }

std::vector<double>& Camera::Params() { return params_; }

double Camera::Params(const size_t idx) const { return params_[idx]; }

double& Camera::Params(const size_t idx) { return params_[idx]; }

const double* Camera::ParamsData() const { return params_.data(); }

double* Camera::ParamsData() { return params_.data(); }

void Camera::SetParams(const std::vector<double>& params) { params_ = params; }

size_t Camera::NumRefracParams() const { return refrac_params_.size(); }

const std::vector<double>& Camera::RefracParams() const {
  return refrac_params_;
}

std::vector<double>& Camera::RefracParams() { return refrac_params_; }

double Camera::RefracParams(const size_t idx) const {
  return refrac_params_[idx];
}

double& Camera::RefracParams(const size_t idx) { return refrac_params_[idx]; }

const double* Camera::RefracParamsData() const { return refrac_params_.data(); }

double* Camera::RefracParamsData() { return refrac_params_.data(); }

void Camera::SetRefracParams(const std::vector<double>& refrac_params) {
  refrac_params_ = refrac_params;
}

}  // namespace colmap
