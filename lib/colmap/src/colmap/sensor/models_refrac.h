#pragma once

#include "colmap/sensor/models.h"
#include "colmap/sensor/ray3d.h"

namespace colmap {

// This file defines several different refractive camera
// models and arbitrary new refractive camera models can be added by the
// following steps:
//
//  1. Add a new struct in this file which implements all the necessary methods.
//  2. Define an unique refrac_model_name and refrac_model_id for the refractive
//  camera model.
//  3. Add refractive camera model to `CAMERA_REFRAC_MODEL_CASES` macro in this
//  file.
//  4. Add new template specialization of test case for refractive camera model
//  to
//     `models_refrac_test.cc`.
//
// A camera model can have three different types of camera parameters: focal
// length, principal point, extra parameters (distortion parameters). The
// parameter array is split into different groups, so that we can enable or
// disable the refinement of the individual groups during bundle adjustment. It
// is up to the camera model to access the parameters correctly (it is free to
// do so in an arbitrary manner) - the parameters are not accessed from outside.
//
// A refractive camera model must have the following methods:
//
//  - `ImgFromCam`: transform camera coordinates to image
//    coordinates (the inverse of `CamFromImg`). Assumes that the camera
//    coordinates are given as (u, v, w).
//  - `CamFromImg`: transform image coordinates to 3D ray in camera
//    coordinates (the inverse of `ImgFromCam`). Produces ray as Ray3D.
//  - `CamFromImgPoint`: transform image coordinates to camera
//    coordinates (the inverse of `ImgFromCam`) given a depth `d`. Produces
//    camera coordinates as (u, v, w).
//
// Whenever you specify the refractive camera parameters in a list, they must
// appear exactly in the order as they are accessed in the defined model struct.
//
// The camera models follow the convention that the upper left image corner has
// the coordinate (0, 0), the lower right corner (width, height), i.e. that
// the upper left pixel center has coordinate (0.5, 0.5) and the lower right
// pixel center has the coordinate (width - 0.5, height - 0.5).

static const int kInvalidRefractiveCameraModelId = -1;

#ifndef CAMERA_REFRAC_MODEL_DEFINITIONS
#define CAMERA_REFRAC_MODEL_DEFINITIONS(                                       \
    refrac_model_id_value, refrac_model_name_value, num_params_value)          \
  static const int kRefracModelId = refrac_model_id_value;                     \
  static const size_t kNumParams = num_params_value;                           \
  static const int refrac_model_id;                                            \
  static const std::string refrac_model_name;                                  \
  static const size_t num_params;                                              \
  static const std::string params_info;                                        \
  static const std::vector<size_t> optimizable_params_idxs;                    \
                                                                               \
  static inline int InitializeRefracModelId() {                                \
    return refrac_model_id_value;                                              \
  };                                                                           \
  static inline std::string InitializeRefracModelName() {                      \
    return refrac_model_name_value;                                            \
  }                                                                            \
  static inline size_t InitializeNumParams() { return num_params_value; }      \
  static inline std::string InitializeRefracModelParamsInfo();                 \
  static inline std::vector<size_t> InitializeOptimizableParamsIdxs();         \
                                                                               \
  template <typename CameraModel, typename T>                                  \
  static void ImgFromCam(                                                      \
      const T* cam_params, const T* refrac_params, T u, T v, T w, T* x, T* y); \
                                                                               \
  template <typename CameraModel, typename T>                                  \
  static void CamFromImg(const T* cam_params,                                  \
                         const T* refrac_params,                               \
                         T x,                                                  \
                         T y,                                                  \
                         Eigen::Matrix<T, 3, 1>* ori,                          \
                         Eigen::Matrix<T, 3, 1>* dir);                         \
  template <typename CameraModel, typename T>                                  \
  static void CamFromImgPoint(const T* cam_params,                             \
                              const T* refrac_params,                          \
                              T x,                                             \
                              T y,                                             \
                              T d,                                             \
                              Eigen::Matrix<T, 3, 1>* uvw);                    \
  template <typename T>                                                        \
  static void RefractionAxis(const T* refrac_params,                           \
                             Eigen::Matrix<T, 3, 1>* refraction_axis);
#endif

#ifndef CAMERA_REFRAC_MODEL_CASES
#define CAMERA_REFRAC_MODEL_CASES    \
  CAMERA_REFRAC_MODEL_CASE(FlatPort) \
  CAMERA_REFRAC_MODEL_CASE(DomePort)
#endif

#ifndef CAMERA_COMBINATION_MODEL_CASES
#define CAMERA_COMBINATION_MODEL_CASES                                    \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, SimplePinholeCameraModel)       \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, PinholeCameraModel)             \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, SimpleRadialCameraModel)        \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, SimpleRadialFisheyeCameraModel) \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, RadialCameraModel)              \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, RadialFisheyeCameraModel)       \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, OpenCVCameraModel)              \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, OpenCVFisheyeCameraModel)       \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, FullOpenCVCameraModel)          \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, FOVCameraModel)                 \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, ThinPrismFisheyeCameraModel)    \
  CAMERA_COMBINATION_MODEL_CASE(FlatPort, MetashapeFisheyeCameraModel)    \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, SimplePinholeCameraModel)       \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, PinholeCameraModel)             \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, SimpleRadialCameraModel)        \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, SimpleRadialFisheyeCameraModel) \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, RadialCameraModel)              \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, RadialFisheyeCameraModel)       \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, OpenCVCameraModel)              \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, OpenCVFisheyeCameraModel)       \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, FullOpenCVCameraModel)          \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, FOVCameraModel)                 \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, ThinPrismFisheyeCameraModel)    \
  CAMERA_COMBINATION_MODEL_CASE(DomePort, MetashapeFisheyeCameraModel)
#endif

#ifndef CAMERA_REFRAC_MODEL_SWITCH_CASES
#define CAMERA_REFRAC_MODEL_SWITCH_CASES         \
  CAMERA_REFRAC_MODEL_CASES                      \
  default:                                       \
    CAMERA_REFRAC_MODEL_DOES_NOT_EXIST_EXCEPTION \
    break;
#endif

#define CAMERA_REFRAC_MODEL_DOES_NOT_EXIST_EXCEPTION \
  throw std::domain_error("Refractive camera model does not exist");

#define CAMERA_COMBINATION_MODEL_DOES_NOT_EXIST_EXCEPTION                     \
  throw std::domain_error(                                                    \
      "The combination of a perspective camera model and a refractive model " \
      "does not exist");

#ifndef CAMERA_COMBINATION_MODEL_IF_ELSE_CASES
#define CAMERA_COMBINATION_MODEL_IF_ELSE_CASES        \
  CAMERA_COMBINATION_MODEL_CASES                      \
                                                      \
  {                                                   \
    CAMERA_COMBINATION_MODEL_DOES_NOT_EXIST_EXCEPTION \
  }
#endif

// The "Curiously Recurring Template Pattern" (CRTP) is
// used here, so that we can reuse some shared
// functionality between all camera models - defined in
// the BaseCameraRefracModel.
template <typename CameraRefracModel>
struct BaseCameraRefracModel {
  template <typename CameraModel, typename T>
  static inline void IterativeProjection(
      const T* cam_params, const T* refrac_params, T u, T v, T w, T* x, T* y);
};

// FlatPort refraction model (thick planar glass interface).
//
// Parameter list is expected in the following order:
//
// Nx, Ny, Nz, int_dist, int_thick, na, ng, nw (Nx, Ny,
// Nz is the planar interface normal in the local
// camera coordinate system)
//
// Note that (Nx, Ny, Nz) must be unit vector, e.g. (0,
// 0, 1) means the plane normal is the same as the
// camera's local Z-axis
//
// see:
// https://link.springer.com/chapter/10.1007/978-3-642-33715-4_61
struct FlatPort : public BaseCameraRefracModel<FlatPort> {
  CAMERA_REFRAC_MODEL_DEFINITIONS(0, "FLATPORT", 8)
};

// DomePort (thick spherical glass interface).
//
// Parameter list is expected in the following order:
//
// Cx, Cy, Cz, int_radius, int_thick, na, ng, nw (Cx,
// Cy, Cz is the spherical center in the local camera
// coordinate system)
//
// see:
// https://link.springer.com/chapter/10.1007/978-3-030-33676-9_6
struct DomePort : public BaseCameraRefracModel<DomePort> {
  CAMERA_REFRAC_MODEL_DEFINITIONS(1, "DOMEPORT", 8)
};

// Check whether refractive camera with given name or identifier
// exists
bool ExistsCameraRefracModelWithName(const std::string& refrac_model_name);
bool ExistsCameraRefracModelWithId(int refrac_model_id);

// Convert refractive camera name to unique refractive camera model identifier.
//
// @param refrac_model_name      Unique name of refractive camera model
//
// @return                       Unique identifier of refractive camera model
int CameraRefracModelNameToId(const std::string& refrac_model_name);

// Convert refractive camera model identifier to unique refractive camera model
// name.
//
// @param refrac_model_id        Unique identifier of refractive camera model
//
// @param                        Unique name of refractive camera model
std::string CameraRefracModelIdToName(int refrac_model_id);

// Get human-readable information about the refractive parameter vector order.
//
// @param refrac_model_id        Unique identifier of refractive camera model
std::string CameraRefracModelParamsInfo(int refrac_model_id);

// Get the indices of the parameter groups in the parameter vector.
//
// @param refrac_model_id     Unique identifier of refractive camera model.
const std::vector<size_t>& CameraRefracModelOptimizableParamsIdxs(
    int refrac_model_id);

// Get the total number of parameters of a refractive camera model
size_t CameraRefracModelNumParams(int refrac_model_id);

// Check whether parameters are valid, i.e. the parameter vector has
// the correct dimensions that match the specified refractive camera model.
//
// @param refrac_model_id      Unique identifier of refractive camera model.
// @param params               Array of camera parameters.
bool CameraRefracModelVerifyParams(int refrac_model_id,
                                   const std::vector<double>& params);

// Transform camera to image coordinates using refractive camera model.
//
// This is the inverse of `CameraRefracModelCamFromImg`.
//
// @param model_id              Unique model_id of camera model as defined in
//                              `CAMERA_MODEL_NAME_TO_CODE`.
// @param refrac_model_id       Unique refrac_model_id of camera model as
// defined in                           `CAMERA_REFRAC_MODEL_NAME_TO_CODE`.
// @param cam_params            Array of camera parameters.
// @param refrac_params         Array of refractive parameters.
// @param u, v, w               Coordinates in camera system as (u, v, w).
// @param x, y                  Output image coordinates in pixels.
inline Eigen::Vector2d CameraRefracModelImgFromCam(
    int model_id,
    int refrac_model_id,
    const std::vector<double>& cam_params,
    const std::vector<double>& refrac_params,
    const Eigen::Vector3d& uvw);

// Transform image coordinates to 3D ray in camera frame using refractive camera
// model.
//
// This is the inverse of `CameraRefracModelImgFromCam`.
//
// @param model_id      Unique identifier of camera model.
// @param refrac_model_id      Unique identifier of refractive camera model.
// @param cam_params        Array of camera parameters.
// @param refrac_params        Array of refractive camera parameters.
// @param xy            Image coordinates in pixels.
//
// @return              Output ray in camera system as Ray3D.
inline Ray3D CameraRefracModelCamFromImg(
    int model_id,
    int refrac_model_id,
    const std::vector<double>& cam_params,
    const std::vector<double>& refrac_params,
    const Eigen::Vector2d& xy);

// Transform image to camera coordinates given a depth `d` using refractive
// camera model.
//
// This is the inverse of `CameraRefracModelImgFromCam`.
//
// @param model_id      Unique identifier of camera model.
// @param refrac_model_id      Unique identifier of refractive camera model.
// @param cam_params        Array of camera parameters.
// @param refrac_params        Array of refractive camera parameters.
// @param xy            Image coordinates in pixels.
// @param d            Depth of the point in the camera system.
//
// @return              Output Coordinates in camera system as (u, v, w).
inline Eigen::Vector3d CameraRefracModelCamFromImgPoint(
    int model_id,
    int refrac_model_id,
    const std::vector<double>& cam_params,
    const std::vector<double>& refrac_params,
    const Eigen::Vector2d& xy,
    const double d);

inline Eigen::Vector3d CameraRefracModelRefractionAxis(
    int refrac_model_id, const std::vector<double>& refrac_params);

////////////////////////////////////////////////////////////////////////////////
// BaseCameraRefracModel

template <typename CameraRefracModel>
template <typename CameraModel, typename T>
void BaseCameraRefracModel<CameraRefracModel>::IterativeProjection(
    const T* cam_params, const T* refrac_params, T u, T v, T w, T* x, T* y) {
  // Parameters for Newton iteration using numerical differentiation with
  // central differences, 100 iterations should be enough even for complex
  // camera models with higher order terms.
  const Eigen::Matrix<T, 3, 1> uvw(u, v, w);
  const T d = uvw.norm();
  const size_t kNumIterations = 100;
  const T kMaxStepNorm = T(1e-10);
  const T kRelStepSize = T(1e-9);
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
    CameraRefracModel::template CamFromImgPoint<CameraModel, T>(
        cam_params, refrac_params, X(0), X(1), d, &err);
    err = err - uvw;
    CameraRefracModel::template CamFromImgPoint<CameraModel, T>(
        cam_params, refrac_params, X(0) - step0, X(1), d, &dx_0b);

    CameraRefracModel::template CamFromImgPoint<CameraModel, T>(
        cam_params, refrac_params, X(0) + step0, X(1), d, &dx_0f);

    CameraRefracModel::template CamFromImgPoint<CameraModel, T>(
        cam_params, refrac_params, X(0), X(1) - step1, d, &dx_1b);

    CameraRefracModel::template CamFromImgPoint<CameraModel, T>(
        cam_params, refrac_params, X(0), X(1) + step1, d, &dx_1f);

    J.col(0) = (dx_0f - dx_0b) / (T(2) * step0);
    J.col(1) = (dx_1f - dx_1b) / (T(2) * step1);
    Eigen::Matrix<T, 2, 2> H = J.transpose() * J;
    Eigen::Matrix<T, 2, 1> b = J.transpose() * err;
    const Eigen::Matrix<T, 2, 1> step_x = H.partialPivLu().solve(b);
    X -= step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) {
      break;
    }
  }
  *x = X(0);
  *y = X(1);
}

////////////////////////////////////////////////////////////////////////////////
// FlatPort

std::string FlatPort::InitializeRefracModelParamsInfo() {
  return "Nx, Ny, Nz, int_dist, int_thick, na, ng, nw\n(Note that [Nx, Ny, Nz] "
         "must be unit vector)";
}

std::vector<size_t> FlatPort::InitializeOptimizableParamsIdxs() {
  return {0, 1, 2, 3};
}

template <typename CameraModel, typename T>
void FlatPort::ImgFromCam(
    const T* cam_params, const T* refrac_params, T u, T v, T w, T* x, T* y) {
  CameraModel::ImgFromCam(cam_params, u, v, w, x, y);
  IterativeProjection<CameraModel, T>(cam_params, refrac_params, u, v, w, x, y);
  return;
}

template <typename CameraModel, typename T>
void FlatPort::CamFromImg(const T* cam_params,
                          const T* refrac_params,
                          const T x,
                          const T y,
                          Eigen::Matrix<T, 3, 1>* ori,
                          Eigen::Matrix<T, 3, 1>* dir) {
  (*ori) = Eigen::Matrix<T, 3, 1>::Zero();
  CameraModel::CamFromImg(cam_params, x, y, &(*dir)(0), &(*dir)(1), &(*dir)(2));
  (*dir).normalize();
  const Eigen::Matrix<T, 3, 1> int_normal(
      refrac_params[0], refrac_params[1], refrac_params[2]);

  const T int_dist = refrac_params[3];
  const T int_thick = refrac_params[4];
  const T na = refrac_params[5];
  const T ng = refrac_params[6];
  const T nw = refrac_params[7];

  T d;
  bool is_intersect =
      RayPlaneIntersection(*ori, *dir, int_normal, int_dist, &d);

  // The back-projected ray has no intersection with the planar interface,
  // continue
  if (!is_intersect) return;
  *ori += d * *dir;

  ComputeRefraction(int_normal, na, ng, dir);

  is_intersect =
      RayPlaneIntersection(*ori, *dir, int_normal, T(int_dist + int_thick), &d);
  *ori += d * *dir;
  ComputeRefraction(int_normal, ng, nw, dir);
  return;
}

template <typename CameraModel, typename T>
void FlatPort::CamFromImgPoint(const T* cam_params,
                               const T* refrac_params,
                               T x,
                               T y,
                               T d,
                               Eigen::Matrix<T, 3, 1>* uvw) {
  Eigen::Matrix<T, 3, 1> ori = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> dir = Eigen::Matrix<T, 3, 1>::Zero();
  CamFromImg<CameraModel, T>(cam_params, refrac_params, x, y, &ori, &dir);
  T lambd1 =
      -(ori.dot(dir) +
        sqrt(-ori[0] * ori[0] * dir[1] * dir[1] -
             ori[0] * ori[0] * dir[2] * dir[2] +
             T(2) * ori[0] * dir[0] * ori[1] * dir[1] +
             T(2) * ori[0] * dir[0] * ori[2] * dir[2] -
             dir[0] * dir[0] * ori[1] * ori[1] -
             dir[0] * dir[0] * ori[2] * ori[2] + dir[0] * dir[0] * d * d -
             ori[1] * ori[1] * dir[2] * dir[2] +
             T(2) * ori[1] * dir[1] * ori[2] * dir[2] -
             dir[1] * dir[1] * ori[2] * ori[2] + dir[1] * dir[1] * d * d +
             dir[2] * dir[2] * d * d)) /
      (dir.dot(dir));

  T lambd2 =
      -(ori.dot(dir) -
        sqrt(-ori[0] * ori[0] * dir[1] * dir[1] -
             ori[0] * ori[0] * dir[2] * dir[2] +
             T(2) * ori[0] * dir[0] * ori[1] * dir[1] +
             T(2) * ori[0] * dir[0] * ori[2] * dir[2] -
             dir[0] * dir[0] * ori[1] * ori[1] -
             dir[0] * dir[0] * ori[2] * ori[2] + dir[0] * dir[0] * d * d -
             ori[1] * ori[1] * dir[2] * dir[2] +
             T(2) * ori[1] * dir[1] * ori[2] * dir[2] -
             dir[1] * dir[1] * ori[2] * ori[2] + dir[1] * dir[1] * d * d +
             dir[2] * dir[2] * d * d)) /
      (dir.dot(dir));

  T lambd;
  if (lambd1 >= T(0)) {
    lambd = lambd1;
  } else {
    lambd = lambd2;
  }

  *uvw = ori + lambd * dir;
}

template <typename T>
void FlatPort::RefractionAxis(const T* refrac_params,
                              Eigen::Matrix<T, 3, 1>* refraction_axis) {
  (*refraction_axis)[0] = refrac_params[0];
  (*refraction_axis)[1] = refrac_params[1];
  (*refraction_axis)[2] = refrac_params[2];
  (*refraction_axis).normalize();
}

////////////////////////////////////////////////////////////////////////////////
// DomePort

std::string DomePort::InitializeRefracModelParamsInfo() {
  return "Cx, Cy, Cz, int_radius, int_thick, na, ng, nw";
}

std::vector<size_t> DomePort::InitializeOptimizableParamsIdxs() {
  return {0, 1, 2};
}

template <typename CameraModel, typename T>
void DomePort::ImgFromCam(
    const T* cam_params, const T* refrac_params, T u, T v, T w, T* x, T* y) {
  CameraModel::ImgFromCam(cam_params, u, v, w, x, y);
  IterativeProjection<CameraModel, T>(cam_params, refrac_params, u, v, w, x, y);
  return;
}

template <typename CameraModel, typename T>
void DomePort::CamFromImg(const T* cam_params,
                          const T* refrac_params,
                          const T x,
                          const T y,
                          Eigen::Matrix<T, 3, 1>* ori,
                          Eigen::Matrix<T, 3, 1>* dir) {
  (*ori) = Eigen::Matrix<T, 3, 1>::Zero();
  CameraModel::CamFromImg(cam_params, x, y, &(*dir)(0), &(*dir)(1), &(*dir)(2));
  (*dir).normalize();
  const Eigen::Matrix<T, 3, 1> sphere_center(
      refrac_params[0], refrac_params[1], refrac_params[2]);
  const T int_radius = refrac_params[3];
  const T int_thick = refrac_params[4];
  const T na = refrac_params[5];
  const T ng = refrac_params[6];
  const T nw = refrac_params[7];

  T dmin, dmax;
  int num_intersects = RaySphereIntersection(
      *ori, *dir, sphere_center, int_radius, &dmin, &dmax);

  // no intersection with sphereical refraction interface
  if (num_intersects == 0) return;
  *ori += dmax * *dir;

  Eigen::Matrix<T, 3, 1> normal = *ori - sphere_center;
  normal.normalize();

  ComputeRefraction(normal, na, ng, dir);

  num_intersects = RaySphereIntersection(
      *ori, *dir, sphere_center, T(int_radius + int_thick), &dmin, &dmax);
  *ori += dmax * *dir;
  normal = *ori - sphere_center;
  normal.normalize();
  ComputeRefraction(normal, ng, nw, dir);

  return;
}

template <typename CameraModel, typename T>
void DomePort::CamFromImgPoint(const T* cam_params,
                               const T* refrac_params,
                               T x,
                               T y,
                               T d,
                               Eigen::Matrix<T, 3, 1>* uvw) {
  Eigen::Matrix<T, 3, 1> ori = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> dir = Eigen::Matrix<T, 3, 1>::Zero();
  CamFromImg<CameraModel, T>(cam_params, refrac_params, x, y, &ori, &dir);
  T lambd1 =
      -(ori.dot(dir) +
        sqrt(-ori[0] * ori[0] * dir[1] * dir[1] -
             ori[0] * ori[0] * dir[2] * dir[2] +
             T(2) * ori[0] * dir[0] * ori[1] * dir[1] +
             T(2) * ori[0] * dir[0] * ori[2] * dir[2] -
             dir[0] * dir[0] * ori[1] * ori[1] -
             dir[0] * dir[0] * ori[2] * ori[2] + dir[0] * dir[0] * d * d -
             ori[1] * ori[1] * dir[2] * dir[2] +
             T(2) * ori[1] * dir[1] * ori[2] * dir[2] -
             dir[1] * dir[1] * ori[2] * ori[2] + dir[1] * dir[1] * d * d +
             dir[2] * dir[2] * d * d)) /
      (dir.dot(dir));

  T lambd2 =
      -(ori.dot(dir) -
        sqrt(-ori[0] * ori[0] * dir[1] * dir[1] -
             ori[0] * ori[0] * dir[2] * dir[2] +
             T(2) * ori[0] * dir[0] * ori[1] * dir[1] +
             T(2) * ori[0] * dir[0] * ori[2] * dir[2] -
             dir[0] * dir[0] * ori[1] * ori[1] -
             dir[0] * dir[0] * ori[2] * ori[2] + dir[0] * dir[0] * d * d -
             ori[1] * ori[1] * dir[2] * dir[2] +
             T(2) * ori[1] * dir[1] * ori[2] * dir[2] -
             dir[1] * dir[1] * ori[2] * ori[2] + dir[1] * dir[1] * d * d +
             dir[2] * dir[2] * d * d)) /
      (dir.dot(dir));

  T lambd;
  if (lambd1 >= T(0)) {
    lambd = lambd1;
  } else {
    lambd = lambd2;
  }

  *uvw = ori + lambd * dir;
}

template <typename T>
void DomePort::RefractionAxis(const T* refrac_params,
                              Eigen::Matrix<T, 3, 1>* refraction_axis) {
  (*refraction_axis)[0] = refrac_params[0];
  (*refraction_axis)[1] = refrac_params[1];
  (*refraction_axis)[2] = refrac_params[2];
  (*refraction_axis).normalize();
}

////////////////////////////////////////////////////////////////////////////////

Eigen::Vector2d CameraRefracModelImgFromCam(
    const int model_id,
    const int refrac_model_id,
    const std::vector<double>& cam_params,
    const std::vector<double>& refrac_params,
    const Eigen::Vector3d& uvw) {
  Eigen::Vector2d xy;
#define CAMERA_COMBINATION_MODEL_CASE(CameraRefracModel, CameraModel) \
  if (model_id == CameraModel::kModelId &&                            \
      refrac_model_id == CameraRefracModel::kRefracModelId) {         \
    CameraRefracModel::ImgFromCam<CameraModel>(cam_params.data(),     \
                                               refrac_params.data(),  \
                                               uvw.x(),               \
                                               uvw.y(),               \
                                               uvw.z(),               \
                                               &xy.x(),               \
                                               &xy.y());              \
  } else

  CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE
  return xy;
}

Ray3D CameraRefracModelCamFromImg(const int model_id,
                                  const int refrac_model_id,
                                  const std::vector<double>& cam_params,
                                  const std::vector<double>& refrac_params,
                                  const Eigen::Vector2d& xy) {
  Ray3D ray;
#define CAMERA_COMBINATION_MODEL_CASE(CameraRefracModel, CameraModel) \
  if (model_id == CameraModel::kModelId &&                            \
      refrac_model_id == CameraRefracModel::kRefracModelId) {         \
    CameraRefracModel::CamFromImg<CameraModel>(cam_params.data(),     \
                                               refrac_params.data(),  \
                                               xy.x(),                \
                                               xy.y(),                \
                                               &ray.ori,              \
                                               &ray.dir);             \
  } else

  CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE
  return ray;
}

Eigen::Vector3d CameraRefracModelCamFromImgPoint(
    const int model_id,
    const int refrac_model_id,
    const std::vector<double>& cam_params,
    const std::vector<double>& refrac_params,
    const Eigen::Vector2d& xy,
    const double d) {
  Eigen::Vector3d uvw;
#define CAMERA_COMBINATION_MODEL_CASE(CameraRefracModel, CameraModel)      \
  if (model_id == CameraModel::kModelId &&                                 \
      refrac_model_id == CameraRefracModel::kRefracModelId) {              \
    CameraRefracModel::CamFromImgPoint<CameraModel>(                       \
        cam_params.data(), refrac_params.data(), xy.x(), xy.y(), d, &uvw); \
  } else

  CAMERA_COMBINATION_MODEL_IF_ELSE_CASES

#undef CAMERA_COMBINATION_MODEL_CASE
  return uvw;
}

Eigen::Vector3d CameraRefracModelRefractionAxis(
    const int refrac_model_id, const std::vector<double>& refrac_params) {
  Eigen::Vector3d refrac_axis;
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)                        \
  case CameraRefracModel::kRefracModelId:                                  \
    CameraRefracModel::RefractionAxis(refrac_params.data(), &refrac_axis); \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }
  return refrac_axis;
}

}  // namespace colmap