#pragma once

#include <colmap/src/base/image.h>
#include <unordered_map>

namespace calibmar {
  // Image holds 2D-3D corresponence and pose information.
  class Image {
   public:
    inline const std::string& Name();
    inline void SetName(std::string& name);

    inline void SetPoints2D(const std::vector<Eigen::Vector2d>& points);
    inline const std::vector<Eigen::Vector2d>& Points2D() const;
    inline Eigen::Vector2d Point2D(size_t idx) const;

    inline void SetPoint3DforPoint2D(const uint32_t point3D_id, const size_t point2D_idx);

    inline const std::unordered_map<size_t, uint32_t>& Correspondences() const;

    // Access quaternion vector as (qw, qx, qy, qz) specifying the rotation of the
    // pose which is defined as the transformation from world to image space.
    inline const Eigen::Vector4d& Rotation() const;
    inline Eigen::Vector4d& Rotation();
    // Set quaternion vector as (qw, qx, qy, qz) specifying the rotation of the
    // pose which is defined as the transformation from world to image space.
    inline void SetRotation(const Eigen::Vector4d& qvec);

    // Access vector as (tx, ty, tz) specifying the translation of the
    // pose which is defined as the transformation from world to image space.
    inline const Eigen::Vector3d& Translation() const;
    inline Eigen::Vector3d& Translation();
    inline void SetTranslation(const Eigen::Vector3d& tvec);

   private:
    std::string name_;
    std::vector<Eigen::Vector2d> points2D_;
    std::unordered_map<size_t, uint32_t> correspondences_;
    Eigen::Vector4d rotation_;
    Eigen::Vector3d translation_;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////

  inline void Image::SetPoints2D(const std::vector<Eigen::Vector2d>& points) {
    points2D_ = points;
  }

  inline Eigen::Vector2d Image::Point2D(size_t idx) const {
    return points2D_.at(idx);
  }

  inline const std::vector<Eigen::Vector2d>& Image::Points2D() const {
    return points2D_;
  }

  inline void Image::SetPoint3DforPoint2D(const uint32_t point3D_id, const size_t point2D_idx) {
    correspondences_[point2D_idx] = point3D_id;
  }

  inline const std::unordered_map<size_t, uint32_t>& Image::Correspondences() const {
    return correspondences_;
  }

  inline const std::string& Image::Name() {
    return name_;
  }

  inline void Image::SetName(std::string& name) {
    name_ = name;
  }

  inline const Eigen::Vector4d& Image::Rotation() const {
    return rotation_;
  }

  inline Eigen::Vector4d& Image::Rotation() {
    return rotation_;
  }

  inline void Image::SetRotation(const Eigen::Vector4d& rotation) {
    rotation_ = rotation;
  }

  inline const Eigen::Vector3d& Image::Translation() const {
    return translation_;
  }

  inline Eigen::Vector3d& Image::Translation() {
    return translation_;
  }

  inline void Image::SetTranslation(const Eigen::Vector3d& translation) {
    translation_ = translation;
  }
}