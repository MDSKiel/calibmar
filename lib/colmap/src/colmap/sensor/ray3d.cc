#include "colmap/sensor/ray3d.h"

namespace colmap {

Ray3D::Ray3D()
    : ori(Eigen::Vector3d::Zero()), dir(Eigen::Vector3d(0.0, 0.0, 1.0)) {}

Ray3D::Ray3D(const Eigen::Vector3d& ori, const Eigen::Vector3d& dir)
    : ori(ori), dir(dir.normalized()) {}

Eigen::Vector3d Ray3D::At(const double distance) const {
  return ori + dir * distance;
}
}  // namespace colmap
