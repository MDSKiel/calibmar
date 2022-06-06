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

#ifndef COLMAP_SRC_BASE_CAMERA_NON_SVP_MODELS_H_
#define COLMAP_SRC_BASE_CAMERA_NON_SVP_MODELS_H_

#include <cfloat>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>

#include "base/camera_models.h"
#include "base/ray3d.h"

namespace colmap {

// This file defines several different Non-Single-View-Point (non-SVP) camera
// models and arbitrary new non-SVP camera models can be added by the following
// steps:
//
//  1. Add a new struct in this file which implements all the necessary methods.
//  2. Define an unique non_svp_model_name and non_svp_model_id for the camera
//  model.
//  3. Add camera model to `CAMERA_NON_SVP_MODEL_CASES` macro in this file.
//  4. Add new template specialization of test case for camera model to
//     `camera_non_svp_models_test.cc`.
//
// A camera model can have three different types of camera parameters: focal
// length, principal point, extra parameters (distortion parameters). The
// parameter array is split into different groups, so that we can enable or
// disable the refinement of the individual groups during bundle adjustment. It
// is up to the camera model to access the parameters correctly (it is free to
// do so in an arbitrary manner) - the parameters are not accessed from outside.
//
// A camera model must have the following methods:
//
//  - `WorldToImage`: transform 3D world point in camera coordinates frame to
//  image
//    coordinates (the inverse of `ImageToWorld`). Assumes that the world
//    coordinates are given as (x, y, z).
//  - `ImageToWorld`: transform image coordinates to 3D ray in camera
//  coordinates frame
// (the inverse of `WorldToImage`). Produces the ray as ray origin and ray unit
// direction
//
//  - `ImageToWorldPoint`: transform image coordinates to 3D world point given
//  depth value
// of this pixel position. Produces the 3D world point in the camera coordinate
// frame (x, y, z).
//
// Whenever you specify the camera parameters in a list, they must appear
// exactly in the order as they are accessed in the defined model struct.
//
// The camera models follow the convention that the upper left image corner has
// the coordinate (0, 0), the lower right corner (width, height), i.e. that
// the upper left pixel center has coordinate (0.5, 0.5) and the lower right
// pixel center has the coordinate (width - 0.5, height - 0.5).

static const int kInvalidCameraNonSvpModelId = -1;

#ifndef CAMERA_NON_SVP_MODEL_DEFINITIONS
#define CAMERA_NON_SVP_MODEL_DEFINITIONS(                                    \
    non_svp_model_id_value, non_svp_model_name_value, num_params_value)      \
  static const int kNonSvpModelId = non_svp_model_id_value;                  \
  static const size_t kNumParams = num_params_value;                         \
  static const int non_svp_model_id;                                         \
  static const std::string non_svp_model_name;                               \
  static const size_t num_params;                                            \
  static const std::string params_info;                                      \
                                                                             \
  static inline int InitializeNonSvpModelId() {                              \
    return non_svp_model_id_value;                                           \
  };                                                                         \
  static inline std::string InitializeNonSvpModelName() {                    \
    return non_svp_model_name_value;                                         \
  }                                                                          \
  static inline size_t InitializeNumParams() { return num_params_value; }    \
  static inline std::string InitializeNonSvpModelParamsInfo();               \
                                                                             \
  template <typename CameraModel, typename T>                                \
  static void ImageToWorld(                                                  \
      const T* non_svp_params, const T* camera_params, const T x, const T y, \
      Eigen::Matrix<T, 3, 1>* ray_ori, Eigen::Matrix<T, 3, 1>* ray_dir);     \
                                                                             \
  template <typename CameraModel, typename T>                                \
  static void WorldToImage(const T* non_svp_params, const T* camera_params,  \
                           const Eigen::Matrix<T, 3, 1>& world_point, T* x,  \
                           T* y);                                            \
                                                                             \
  template <typename CameraModel, typename T>                                \
  static void ImageToWorldPoint(                                             \
      const T* non_svp_params, const T* camera_params, const T x, const T y, \
      const T depth, Eigen::Matrix<T, 3, 1>* world_point);
#endif

#ifndef CAMERA_NON_SVP_MODEL_CASES
#define CAMERA_NON_SVP_MODEL_CASES                                   \
  CAMERA_NON_SVP_MODEL_CASE(DoubleLayerSphericalRefractiveInterface) \
  CAMERA_NON_SVP_MODEL_CASE(DoubleLayerPlanarRefractiveInterface)
#endif

#ifndef CAMERA_COMBINATION_MODEL_CASES
#define CAMERA_COMBINATION_MODEL_CASES                                   \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                SimplePinholeCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                PinholeCameraModel)                      \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                SimpleRadialCameraModel)                 \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                SimpleRadialFisheyeCameraModel)          \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                RadialCameraModel)                       \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                RadialFisheyeCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                OpenCVCameraModel)                       \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                OpenCVFisheyeCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                FullOpenCVCameraModel)                   \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                FOVCameraModel)                          \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                ThinPrismFisheyeCameraModel)             \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerSphericalRefractiveInterface, \
                                OpenCV3RadialCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                SimplePinholeCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                PinholeCameraModel)                      \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                SimpleRadialCameraModel)                 \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                SimpleRadialFisheyeCameraModel)          \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                RadialCameraModel)                       \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                RadialFisheyeCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                OpenCVCameraModel)                       \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                OpenCVFisheyeCameraModel)                \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                FullOpenCVCameraModel)                   \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                FOVCameraModel)                          \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                ThinPrismFisheyeCameraModel)             \
  CAMERA_COMBINATION_MODEL_CASE(DoubleLayerPlanarRefractiveInterface,    \
                                OpenCV3RadialCameraModel)
#endif

#ifndef CAMERA_NON_SVP_MODEL_SWITCH_CASES
#define CAMERA_NON_SVP_MODEL_SWITCH_CASES         \
  CAMERA_NON_SVP_MODEL_CASES                      \
  default:                                        \
    CAMERA_NON_SVP_MODEL_DOES_NOT_EXIST_EXCEPTION \
    break;
#endif

#define CAMERA_NON_SVP_MODEL_DOES_NOT_EXIST_EXCEPTION \
  throw std::domain_error("Camera Non-SVP model does not exist");

#define CAMERA_COMBINATION_MODEL_DOES_NOT_EXIST_EXCEPTION \
  throw std::domain_error(                                \
      "The combination of Camera Non-SVP model and SVP model does not exist");

#ifndef CAMERA_COMBINATION_MODEL_IF_ELSE_CASES
#define CAMERA_COMBINATION_MODEL_IF_ELSE_CASES        \
  CAMERA_COMBINATION_MODEL_CASES                      \
                                                      \
  {                                                   \
    CAMERA_COMBINATION_MODEL_DOES_NOT_EXIST_EXCEPTION \
  }
#endif

// The "Curiously Recurring Template Pattern" (CRTP) is used here, so that we
// can reuse some shared functionality between all camera models -
// defined in the BaseCameraModel.
template <typename CameraNonSvpModel>
struct BaseCameraNonSvpModel {
  template <typename CameraModel, typename T>
  static inline void IterativeProjection(
      const T* non_svp_params, const T* camera_params,
      const Eigen::Matrix<T, 3, 1>& world_point, T* x, T* y);
};

// DoubleLayerSphericalRefractiveInterface (thick dome port camera system).
//
// Parameter list is expected in the following order:
//
// Cx, Cy, Cz, int_radius, int_thick, na, ng, nw (Cx, Cy, Cz is the spherical
// center in the local camera coordinate system)
//
// see: https://link.springer.com/chapter/10.1007/978-3-030-33676-9_6
struct DoubleLayerSphericalRefractiveInterface
    : public BaseCameraNonSvpModel<DoubleLayerSphericalRefractiveInterface> {
  CAMERA_NON_SVP_MODEL_DEFINITIONS(0, "THICK_SPHERICAL_REFRACTION", 8)
};

// DoubleLayerPlanarRefractiveInterface (thick flat port camera system).
//
// Parameter list is expected in the following order:
//
// Nx, Ny, Nz, int_dist, int_thick, na, ng, nw (Nx, Ny, Nz is the planar
// interface normal in the local camera coordinate system)
//
// Note that (Nx, Ny, Nz) must be unit vector
//
// see: https://link.springer.com/chapter/10.1007/978-3-642-33715-4_61
struct DoubleLayerPlanarRefractiveInterface
    : public BaseCameraNonSvpModel<DoubleLayerPlanarRefractiveInterface> {
  CAMERA_NON_SVP_MODEL_DEFINITIONS(1, "THICK_PLANAR_REFRACTION", 8)
};

// Check whether non-single-view-point camera with given name or identifier
// exists
bool ExistsCameraNonSvpModelWithName(const std::string& non_svp_model_name);
bool ExistsCameraNonSvpModelWithId(const int non_svp_model_id);

// Covnert camera non-single-view-point model to unique model identifier
//
// @param name      Unique name of non-SVP camera model
//
// @return          Unique identifier of non-SVP camera model
int CameraNonSvpModelNameToId(const std::string& non_svp_model_name);

// Convert camera non-single-view-point model unique identifier to name
//
// @param model_id  Unique identifier of non-SVP camera model
//
// @param           Unique name of non-SVP camera model
std::string CameraNonSvpModelIdToName(const int non_svp_model_id);

// Get human-readable information about the non-SVP parameter vector order
//
// @param model_id   Unique identifier of non-SVP camera model
std::string CameraNonSvpModelParamsInfo(const int non_svp_model_id);

// Get the total number of parameters of a non-SVP camera model
//
// @param       Unique identifier of non-SVP camera model
size_t CameraNonSvpModelNumParams(const int non_svp_model_id);

// Check whether parameters are valid, i.e. the parameter vector has
// the correct dimensions that match the specified non-SVP camera model.
//
// @param model_id      Unique identifier of non-SVP camera model.
// @param params        Array of camera parameters.
bool CameraNonSvpModelVerifyParams(const int non_svp_model_id,
                                   const std::vector<double>& params);

// Transform image coordinates to 3D ray in camera coordinate system using
// Non-SVP camera model.
//
// This is the inverse of `CameraNonSvpModelWorldToImage`.
//
// @param model_id              Unique identifier of camera model.
// @param non_svp_model_id      Unique identifier of non-SVP camera model
// @param camera_params         Array of camera parameters.
// @param non_svp_params        Array of non-SVP model parameters.
// @param x, y                  Image coordinates in pixels.
// @param ray_ori, ray_dir      Output ray in camera system as origin and unit
// direction.
inline void CameraNonSvpModelImageToWorld(
    const int model_id, const int non_svp_model_id,
    const std::vector<double>& camera_params,
    const std::vector<double>& non_svp_params, const double x, const double y,
    Eigen::Vector3d* ray_ori, Eigen::Vector3d* ray_dir);

// Transform image coordinates to 3D point in camera coordinate system using
// Non-SVP camera model given depth value.
//
// @param model_id              Unique identifier of camera model.
// @param non_svp_model_id      Unique identifier of non-SVP camera model
// @param camera_params         Array of camera parameters.
// @param non_svp_params        Array of non-SVP model parameters.
// @param x, y                  Image coordinates in pixels.
// @param depth                  depth: distance from 3D point to the camera
// @param world_point           Output 3D point in camera system
// direction.
inline void CameraNonSvpModelImageToWorldPoint(
    const int model_id, const int non_svp_model_id,
    const std::vector<double>& camera_params,
    const std::vector<double>& non_svp_params, const double x, const double y,
    const double depth, Eigen::Vector3d* world_point);

// Transform world coordinates in camera coordinate system to image coordinates.
//
// This is the inverse of `CameraNonSvpModelImageToWorld`.
//
// @param model_id              Unique model_id of camera model as defined in
//                              `CAMERA_MODEL_NAME_TO_CODE`.
// @param non_svp_model_id      Unique identifier of non-SVP camera model
// @param camera_params         Array of camera parameters.
// @param non_svp_params        Array of non-SVP model parameters.
// @param world_point           Coordinates in camera system as (x, y, z).
// @param x, y                  Output image coordinates in pixels.
inline void CameraNonSvpModelWorldToImage(
    const int model_id, const int non_svp_model_id,
    const std::vector<double>& camera_params,
    const std::vector<double>& non_svp_params,
    const Eigen::Vector3d& world_point, double* x, double* y);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// BaseCameraNonSvpModel

template <typename CameraNonSvpModel>
template <typename CameraModel, typename T>
void BaseCameraNonSvpModel<CameraNonSvpModel>::IterativeProjection(
    const T* non_svp_params, const T* camera_params,
    const Eigen::Matrix<T, 3, 1>& world_point, T* x, T* y) {
  // Parameters for Newton iteration using numerical differentiation with
  // central differences, 100 iterations should be enough even for complex
  // camera models with higher order terms.
  const T depth = world_point.norm();
  const size_t kNumIterations = 100;
  const T kMaxStepNorm = T(1e-10);
  const T kRelStepSize = T(1e-7);
  const T kAbsStepSize = T(1e-6);

  Eigen::Matrix<T, 3, 2> J;
  const Eigen::Matrix<T, 2, 1> X0(*x, *y);
  Eigen::Matrix<T, 2, 1> X(*x, *y);
  Eigen::Matrix<T, 3, 1> err;
  Eigen::Matrix<T, 3, 1> dx_0b;
  Eigen::Matrix<T, 3, 1> dx_0f;
  Eigen::Matrix<T, 3, 1> dx_1b;
  Eigen::Matrix<T, 3, 1> dx_1f;

  for (size_t i = 0; i < kNumIterations; ++i) {
    const T step0 = std::max(kAbsStepSize, ceres::abs(kRelStepSize * X(0)));
    const T step1 = std::max(kAbsStepSize, ceres::abs(kRelStepSize * X(1)));
    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, X(0), X(1), depth, &err);
    err = world_point - err;
    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, X(0) - step0, X(1), depth, &dx_0b);

    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, X(0) + step0, X(1), depth, &dx_0f);

    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, X(0), X(1) - step1, depth, &dx_1b);

    CameraNonSvpModel::template ImageToWorldPoint<CameraModel, T>(
        non_svp_params, camera_params, X(0), X(1) + step1, depth, &dx_1f);

    J.col(0) = (dx_0b - dx_0f) / (T(2.0) * step0);
    J.col(1) = (dx_1b - dx_1f) / (T(2.0) * step1);
    Eigen::Matrix<T, 2, 2> H = J.transpose() * J;
    Eigen::Matrix<T, 2, 1> b = T(-1.0) * J.transpose() * err;
    const Eigen::Matrix<T, 2, 1> step_x = H.ldlt().solve(b);
    X += step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) break;
  }
  *x = X(0);
  *y = X(1);
}

////////////////////////////////////////////////////////////////////////////////
// DoubleLayerSphericalRefractiveInterface
std::string
DoubleLayerSphericalRefractiveInterface::InitializeNonSvpModelParamsInfo() {
  return "Cx, Cy, Cz, int_radius, int_thick, na, ng, nw";
}

template <typename CameraModel, typename T>
void DoubleLayerSphericalRefractiveInterface::ImageToWorld(
    const T* non_svp_params, const T* camera_params, const T x, const T y,
    Eigen::Matrix<T, 3, 1>* ray_ori, Eigen::Matrix<T, 3, 1>* ray_dir) {
  CameraModel::ImageToWorld(camera_params, x, y, &(*ray_dir)(0),
                            &(*ray_dir)(1));
  (*ray_dir).normalize();
  const Eigen::Matrix<T, 3, 1> sphere_center(
      non_svp_params[0], non_svp_params[1], non_svp_params[2]);
  const T int_radius = non_svp_params[3];
  const T int_thick = non_svp_params[4];
  const T na = non_svp_params[5];
  const T ng = non_svp_params[6];
  const T nw = non_svp_params[7];

  T dmin, dmax;
  int num_intersects = RaySphereIntersection(*ray_ori, *ray_dir, sphere_center,
                                             int_radius, &dmin, &dmax);

  // no intersection with sphereical refraction interface
  if (num_intersects == 0) return;
  *ray_ori += dmax * *ray_dir;

  Eigen::Matrix<T, 3, 1> normal = sphere_center - *ray_ori;
  normal.normalize();

  ComputeRefraction(normal, na, ng, ray_dir);

  num_intersects =
      RaySphereIntersection(*ray_ori, *ray_dir, sphere_center,
                            T(int_radius + int_thick), &dmin, &dmax);
  *ray_ori += dmax * *ray_dir;
  normal = sphere_center - *ray_ori;
  normal.normalize();
  ComputeRefraction(normal, ng, nw, ray_dir);

  return;
}

template <typename CameraModel, typename T>
void DoubleLayerSphericalRefractiveInterface::ImageToWorldPoint(
    const T* non_svp_params, const T* camera_params, const T x, const T y,
    const T depth, Eigen::Matrix<T, 3, 1>* world_point) {
  Eigen::Matrix<T, 3, 1> ray_ori = Eigen::Matrix<T, 3, 1>::Zero();

  Eigen::Matrix<T, 3, 1> ray_dir = Eigen::Matrix<T, 3, 1>::Zero();
  ray_dir(2) = T(1.0);
  ImageToWorld<CameraModel, T>(non_svp_params, camera_params, x, y, &ray_ori,
                               &ray_dir);
  T lambd1 = -(ray_ori.dot(ray_dir) +
               sqrt(-ray_ori[0] * ray_ori[0] * ray_dir[1] * ray_dir[1] -
                    ray_ori[0] * ray_ori[0] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[1] * ray_dir[1] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[2] * ray_dir[2] -
                    ray_dir[0] * ray_dir[0] * ray_ori[1] * ray_ori[1] -
                    ray_dir[0] * ray_dir[0] * ray_ori[2] * ray_ori[2] +
                    ray_dir[0] * ray_dir[0] * depth * depth -
                    ray_ori[1] * ray_ori[1] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[1] * ray_dir[1] * ray_ori[2] * ray_dir[2] -
                    ray_dir[1] * ray_dir[1] * ray_ori[2] * ray_ori[2] +
                    ray_dir[1] * ray_dir[1] * depth * depth +
                    ray_dir[2] * ray_dir[2] * depth * depth)) /
             (ray_dir.dot(ray_dir));

  T lambd2 = -(ray_ori.dot(ray_dir) -
               sqrt(-ray_ori[0] * ray_ori[0] * ray_dir[1] * ray_dir[1] -
                    ray_ori[0] * ray_ori[0] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[1] * ray_dir[1] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[2] * ray_dir[2] -
                    ray_dir[0] * ray_dir[0] * ray_ori[1] * ray_ori[1] -
                    ray_dir[0] * ray_dir[0] * ray_ori[2] * ray_ori[2] +
                    ray_dir[0] * ray_dir[0] * depth * depth -
                    ray_ori[1] * ray_ori[1] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[1] * ray_dir[1] * ray_ori[2] * ray_dir[2] -
                    ray_dir[1] * ray_dir[1] * ray_ori[2] * ray_ori[2] +
                    ray_dir[1] * ray_dir[1] * depth * depth +
                    ray_dir[2] * ray_dir[2] * depth * depth)) /
             (ray_dir.dot(ray_dir));

  T lambd;
  if (lambd1 >= T(0)) {
    lambd = lambd1;
  } else {
    lambd = lambd2;
  }

  *world_point = ray_ori + lambd * ray_dir;
}

template <typename CameraModel, typename T>
void DoubleLayerSphericalRefractiveInterface::WorldToImage(
    const T* non_svp_params, const T* camera_params,
    const Eigen::Matrix<T, 3, 1>& world_point, T* x, T* y) {
  const T u = world_point(0) / world_point(2);
  const T v = world_point(1) / world_point(2);
  CameraModel::WorldToImage(camera_params, u, v, x, y);
  IterativeProjection<CameraModel, T>(non_svp_params, camera_params,
                                      world_point, x, y);
  return;
}

////////////////////////////////////////////////////////////////////////////////
// DoubleLayerPlanarRefractiveInterface
std::string
DoubleLayerPlanarRefractiveInterface::InitializeNonSvpModelParamsInfo() {
  return "Nx, Ny, Nz, int_dist, int_thick, na, ng, nw\n(Note that [Nx, Ny, Nz] "
         "must be unit vector)";
}

template <typename CameraModel, typename T>
void DoubleLayerPlanarRefractiveInterface::ImageToWorld(
    const T* non_svp_params, const T* camera_params, const T x, const T y,
    Eigen::Matrix<T, 3, 1>* ray_ori, Eigen::Matrix<T, 3, 1>* ray_dir) {
  CameraModel::ImageToWorld(camera_params, x, y, &(*ray_dir)(0),
                            &(*ray_dir)(1));
  (*ray_dir).normalize();
  const Eigen::Matrix<T, 3, 1> int_normal(non_svp_params[0], non_svp_params[1],
                                          non_svp_params[2]);
  const T int_dist = non_svp_params[3];
  const T int_thick = non_svp_params[4];
  const T na = non_svp_params[5];
  const T ng = non_svp_params[6];
  const T nw = non_svp_params[7];

  T d;
  bool is_intersect =
      RayPlaneIntersection(*ray_ori, *ray_dir, int_normal, int_dist, &d);

  // The back-projected ray has no intersection with the planar interface,
  // continue
  if (!is_intersect) return;
  *ray_ori += d * *ray_dir;

  ComputeRefraction(int_normal, na, ng, ray_dir);

  is_intersect = RayPlaneIntersection(*ray_ori, *ray_dir, int_normal,
                                      T(int_dist + int_thick), &d);
  *ray_ori += d * *ray_dir;
  ComputeRefraction(int_normal, ng, nw, ray_dir);
  return;
}

template <typename CameraModel, typename T>
void DoubleLayerPlanarRefractiveInterface::ImageToWorldPoint(
    const T* non_svp_params, const T* camera_params, const T x, const T y,
    const T depth, Eigen::Matrix<T, 3, 1>* world_point) {
  Eigen::Matrix<T, 3, 1> ray_ori = Eigen::Matrix<T, 3, 1>::Zero();

  Eigen::Matrix<T, 3, 1> ray_dir = Eigen::Matrix<T, 3, 1>::Zero();
  ray_dir(2) = T(1.0);
  ImageToWorld<CameraModel, T>(non_svp_params, camera_params, x, y, &ray_ori,
                               &ray_dir);
  T lambd1 = -(ray_ori.dot(ray_dir) +
               sqrt(-ray_ori[0] * ray_ori[0] * ray_dir[1] * ray_dir[1] -
                    ray_ori[0] * ray_ori[0] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[1] * ray_dir[1] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[2] * ray_dir[2] -
                    ray_dir[0] * ray_dir[0] * ray_ori[1] * ray_ori[1] -
                    ray_dir[0] * ray_dir[0] * ray_ori[2] * ray_ori[2] +
                    ray_dir[0] * ray_dir[0] * depth * depth -
                    ray_ori[1] * ray_ori[1] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[1] * ray_dir[1] * ray_ori[2] * ray_dir[2] -
                    ray_dir[1] * ray_dir[1] * ray_ori[2] * ray_ori[2] +
                    ray_dir[1] * ray_dir[1] * depth * depth +
                    ray_dir[2] * ray_dir[2] * depth * depth)) /
             (ray_dir.dot(ray_dir));

  T lambd2 = -(ray_ori.dot(ray_dir) -
               sqrt(-ray_ori[0] * ray_ori[0] * ray_dir[1] * ray_dir[1] -
                    ray_ori[0] * ray_ori[0] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[1] * ray_dir[1] +
                    T(2) * ray_ori[0] * ray_dir[0] * ray_ori[2] * ray_dir[2] -
                    ray_dir[0] * ray_dir[0] * ray_ori[1] * ray_ori[1] -
                    ray_dir[0] * ray_dir[0] * ray_ori[2] * ray_ori[2] +
                    ray_dir[0] * ray_dir[0] * depth * depth -
                    ray_ori[1] * ray_ori[1] * ray_dir[2] * ray_dir[2] +
                    T(2) * ray_ori[1] * ray_dir[1] * ray_ori[2] * ray_dir[2] -
                    ray_dir[1] * ray_dir[1] * ray_ori[2] * ray_ori[2] +
                    ray_dir[1] * ray_dir[1] * depth * depth +
                    ray_dir[2] * ray_dir[2] * depth * depth)) /
             (ray_dir.dot(ray_dir));

  T lambd;
  if (lambd1 >= T(0)) {
    lambd = lambd1;
  } else {
    lambd = lambd2;
  }

  *world_point = ray_ori + lambd * ray_dir;
}

template <typename CameraModel, typename T>
void DoubleLayerPlanarRefractiveInterface::WorldToImage(
    const T* non_svp_params, const T* camera_params,
    const Eigen::Matrix<T, 3, 1>& world_point, T* x, T* y) {
  const T u = world_point(0) / world_point(2);
  const T v = world_point(1) / world_point(2);
  CameraModel::WorldToImage(camera_params, u, v, x, y);
  IterativeProjection<CameraModel, T>(non_svp_params, camera_params,
                                      world_point, x, y);
  return;
}

////////////////////////////////////////////////////////////////////////////////

void CameraNonSvpModelImageToWorld(const int model_id,
                                   const int non_svp_model_id,
                                   const std::vector<double>& camera_params,
                                   const std::vector<double>& non_svp_params,
                                   const double x, const double y,
                                   Eigen::Vector3d* ray_ori,
                                   Eigen::Vector3d* ray_dir) {
#define CAMERA_COMBINATION_MODEL_CASE(CameraNonSvpModel, CameraModel)         \
  if (model_id == CameraModel::kModelId &&                                    \
      non_svp_model_id == CameraNonSvpModel::kNonSvpModelId) {                \
    CameraNonSvpModel::ImageToWorld<CameraModel>(                             \
        non_svp_params.data(), camera_params.data(), x, y, ray_ori, ray_dir); \
  } else

  CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE
}

void CameraNonSvpModelImageToWorldPoint(
    const int model_id, const int non_svp_model_id,
    const std::vector<double>& camera_params,
    const std::vector<double>& non_svp_params, const double x, const double y,
    const double depth, Eigen::Vector3d* world_point) {
#define CAMERA_COMBINATION_MODEL_CASE(CameraNonSvpModel, CameraModel)          \
  if (model_id == CameraModel::kModelId &&                                     \
      non_svp_model_id == CameraNonSvpModel::kNonSvpModelId) {                 \
    CameraNonSvpModel::ImageToWorldPoint<CameraModel>(non_svp_params.data(),   \
                                                      camera_params.data(), x, \
                                                      y, depth, world_point);  \
  } else

  CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE
}

void CameraNonSvpModelWorldToImage(const int model_id,
                                   const int non_svp_model_id,
                                   const std::vector<double>& camera_params,
                                   const std::vector<double>& non_svp_params,
                                   const Eigen::Vector3d& world_point,
                                   double* x, double* y) {
#define CAMERA_COMBINATION_MODEL_CASE(CameraNonSvpModel, CameraModel)    \
  if (model_id == CameraModel::kModelId &&                               \
      non_svp_model_id == CameraNonSvpModel::kNonSvpModelId) {           \
    CameraNonSvpModel::WorldToImage<CameraModel>(                        \
        non_svp_params.data(), camera_params.data(), world_point, x, y); \
  } else

  CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE
}

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_CAMERA_NON_SVP_MODELS_H_