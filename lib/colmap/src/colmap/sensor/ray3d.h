#pragma once

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace colmap {

// 3D ray class to holds the origin and the direction of a ray in 3D
struct Ray3D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Ray3D();
  Ray3D(const Eigen::Vector3d& ori, const Eigen::Vector3d& dir);

  Eigen::Vector3d At(double distance) const;

  // The 3D position of the ray origion
  Eigen::Vector3d ori;

  // The 3D direction vector of the ray, the direction vector has a unit length
  Eigen::Vector3d dir;
};

// Compute ray refraction accroding to Snell's law
// (note that the total reflection case is not handled here, if there is a total
// reflection event happening, the refracted ray will become (-nan -nan -nan))
//
// @param: normal is the normal vector of the interface which points from the
// interface towards the side of the outgoing ray.
// @param: v is the incident ray and the refracted ray
template <typename T>
inline void ComputeRefraction(const Eigen::Matrix<T, 3, 1>& normal,
                              T n1,
                              T n2,
                              Eigen::Matrix<T, 3, 1>* v);

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
                                 T r,
                                 T* dmin,
                                 T* dmax);

// Compute the intersection of a 3D sphere with a 3D ray
//
// @return: whether the ray intersect the plane
//
// note that the ray direction and the plane normal are assumed to be normalized
template <typename T>
inline bool RayPlaneIntersection(const Eigen::Matrix<T, 3, 1>& ray_ori,
                                 const Eigen::Matrix<T, 3, 1>& ray_dir,
                                 const Eigen::Matrix<T, 3, 1>& normal,
                                 T dist,
                                 T* d);

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
inline void ComputeRefraction(const Eigen::Matrix<T, 3, 1>& normal,
                              T n1,
                              T n2,
                              Eigen::Matrix<T, 3, 1>* v) {
  if (n1 == n2) return;

  const T r = n1 / n2;
  const T c = normal.dot(*v);
  const T scale = (r * c - sqrt(T(1.0) - r * r * (T(1.0) - c * c)));
  *v = r * *v - scale * normal;
  (*v).normalize();
  return;
}

// Implementation from
// https://github.com/g-truc/glm/blob/master/glm/gtx/intersect.inl
template <typename T>
inline int RaySphereIntersection(const Eigen::Matrix<T, 3, 1>& ray_ori,
                                 const Eigen::Matrix<T, 3, 1>& ray_dir,
                                 const Eigen::Matrix<T, 3, 1>& center,
                                 T r,
                                 T* dmin,
                                 T* dmax) {
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
                                 T dist,
                                 T* d) {
  const Eigen::Matrix<T, 3, 1> p0 = dist * normal;
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

// https://web.archive.org/web/20111008212356/http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm
template <typename T>
inline bool IntersectLinesWithTolerance(const Eigen::Matrix<T, 3, 1>& origin1,
                                        const Eigen::Matrix<T, 3, 1>& dir1,
                                        const Eigen::Matrix<T, 3, 1>& origin2,
                                        const Eigen::Matrix<T, 3, 1>& dir2,
                                        Eigen::Matrix<T, 3, 1>& intersection,
                                        T tolerance = T(1e-8)) {
  // from (Pa - Pb).dot(v1) = 0 and (Pa - Pb).dot(v2) = 0
  Eigen::Matrix<T, 3, 1> o = origin1 - origin2;
  T a = dir1.dot(dir1);
  T b = dir1.dot(dir2);
  T c = dir2.dot(dir2);
  T d = dir1.dot(o);
  T e = dir2.dot(o);
  T denom = b * b - a * c;
  if (ceres::abs(denom) <= std::numeric_limits<T>::epsilon()) {
    // lines are paralllel
    intersection = origin1;
    return true;
  }
  T s = (c * d - b * e) / denom;
  T t = (b * d - a * e) / denom;
  Eigen::Matrix<T, 3, 1> Pa = origin1 + s * dir1;
  Eigen::Matrix<T, 3, 1> Pb = origin2 + t * dir2;
  if ((Pb - Pa).norm() >= tolerance) {
    // no intersection
    return false;
  }
  intersection = Pa + (Pb - Pa) / T(2);
  return true;
}

}  // namespace colmap
