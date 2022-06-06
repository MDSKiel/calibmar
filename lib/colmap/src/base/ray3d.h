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

#ifndef COLMAP_SRC_BASE_RAY3D_H_
#define COLMAP_SRC_BASE_RAY3D_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "util/alignment.h"

namespace colmap {

// 3D ray class to holds the origin and the direction of a ray in 3D
struct Ray3D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Ray3D();
  Ray3D(const Eigen::Vector3d& ori, const Eigen::Vector3d& dir);

  Eigen::Vector3d At(const double distance);

  // The 3D position of the ray origion
  Eigen::Vector3d ori;

  // The 3D direction vector of the ray, the direction vector has a unit length
  Eigen::Vector3d dir;
};

// Compute ray refraction accroding to Snell's law
// (note that the total reflection case is not handled here, if there is total
// reflection happens, the refracted ray will become (-nan -nan -nan))
//
// @param: normal is the normal vector of the interface which points from the
// interface towards the side where the light is coming from
// @param: v is the incident ray and the refracted ray
template <typename T>
inline void ComputeRefraction(const Eigen::Matrix<T, 3, 1>& normal, const T n1,
                              const T n2, Eigen::Matrix<T, 3, 1>* v);

// Compute the intersection of a 3D sphere with a 3D ray
//
// @return: the number of intersection points (0: no intersection, 1: 1 the ray
// is tangent to the sphere, 2: the ray intersects the sphere)
//
// note that the ray direction is assumed to be normalized
template <typename T>
inline int RaySphereIntersection(const Eigen::Matrix<T, 3, 1>& ray_ori,
                                 const Eigen::Matrix<T, 3, 1>& ray_dir,
                                 const Eigen::Matrix<T, 3, 1>& center,
                                 const T r, T* dmin, T* dmax);

// Compute the intersection of a 3D sphere with a 3D ray
//
// @return: whether the ray intersect the plane
//
// note that the ray direction and the plane normal are assumed to be normalized
template <typename T>
inline bool RayPlaneIntersection(const Eigen::Matrix<T, 3, 1>& ray_ori,
                                 const Eigen::Matrix<T, 3, 1>& ray_dir,
                                 const Eigen::Matrix<T, 3, 1>& normal,
                                 const T dist, T* d);

// Compute the shortest distance of a 3D point to a ray
//
// implementation see:
// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
template <typename T>
inline T PointToRayDistance(const Eigen::Matrix<T, 3, 1>& point,
                               const Eigen::Matrix<T, 3, 1>& ray_ori,
                               const Eigen::Matrix<T, 3, 1>& ray_dir);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T>
inline void ComputeRefraction(const Eigen::Matrix<T, 3, 1>& normal, const T n1,
                              const T n2, Eigen::Matrix<T, 3, 1>* v) {
  if (n1 == n2) return;

  const T r = n1 / n2;
  const T c = -(normal.dot(*v));
  const T scale = (r * c - sqrt(T(1.0) - r * r * (T(1.0) - c * c)));
  *v = r * *v + scale * normal;
  (*v).normalize();
  return;
}

// Implementation from
// https://github.com/g-truc/glm/blob/master/glm/gtx/intersect.inl
template <typename T>
inline int RaySphereIntersection(const Eigen::Matrix<T, 3, 1>& ray_ori,
                                 const Eigen::Matrix<T, 3, 1>& ray_dir,
                                 const Eigen::Matrix<T, 3, 1>& center,
                                 const T r, T* dmin, T* dmax) {
  const Eigen::Matrix<T, 3, 1> diff = center - ray_ori;
  const T t0 = diff.dot(ray_dir);
  const T d_squared = diff.dot(diff) - t0 * t0;
  if (d_squared > r * r) return false;
  T t1 = sqrt(r * r - d_squared);
  *dmin = t0 - t1;
  *dmax = t0 + t1;
  return 2;
}

template <typename T>
inline bool RayPlaneIntersection(const Eigen::Matrix<T, 3, 1>& ray_ori,
                                 const Eigen::Matrix<T, 3, 1>& ray_dir,
                                 const Eigen::Matrix<T, 3, 1>& normal,
                                 const T dist, T* d) {
  const Eigen::Matrix<T, 3, 1> p0 = T(-1.0) * dist * normal;
  const T denom = ray_dir.dot(normal);
  if (ceres::abs(denom) < std::numeric_limits<T>::epsilon()) return false;

  *d = (p0 - ray_ori).dot(normal) / denom;
  return true;
}

template <typename T>
inline T PointToRayDistance(const Eigen::Matrix<T, 3, 1>& point,
                               const Eigen::Matrix<T, 3, 1>& ray_ori,
                               const Eigen::Matrix<T, 3, 1>& ray_dir) {
  const T t = (point - ray_ori).dot(ray_dir);
  Eigen::Matrix<T, 3, 1> point_closest = ray_ori + t * ray_dir;
  return (point_closest - point).norm();
}

}  // namespace colmap

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(colmap::Ray3D)

#endif  // COLMAP_SRC_BASE_RAY3D_H_
