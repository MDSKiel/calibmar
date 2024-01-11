#pragma once

#include <colmap/feature/types.h>
#include <colmap/scene/image.h>
#include <unordered_map>

namespace calibmar {
  // Image is a single view of a calibration and holds e.g. 2D-3D corresponence and pose information.
  class Image {
   public:
    inline const std::string& Name() const;
    inline void SetName(const std::string& name);

    inline void SetPoints2D(const std::vector<Eigen::Vector2d>& points);
    inline const std::vector<Eigen::Vector2d>& Points2D() const;
    inline Eigen::Vector2d Point2D(size_t idx) const;

    inline void SetPoint3DforPoint2D(const uint32_t point3D_id, const size_t point2D_idx);
    // Correspondences of image 2D point indices to calibration 3D point ids
    inline const std::unordered_map<size_t, uint32_t>& Correspondences() const;

    // Access quaternion specifying the rotation of the
    // pose which is defined as the transformation from world to image space.
    inline const Eigen::Quaterniond& Rotation() const;
    inline Eigen::Quaterniond& Rotation();
    // Set quaternion specifying the rotation of the
    // pose which is defined as the transformation from world to image space.
    inline void SetRotation(const Eigen::Quaterniond& quat);

    // Access vector as (tx, ty, tz) specifying the translation of the
    // pose which is defined as the transformation from world to image space.
    inline const Eigen::Vector3d& Translation() const;
    inline Eigen::Vector3d& Translation();
    inline void SetTranslation(const Eigen::Vector3d& tvec);

    inline void SetDescriptors(const colmap::FeatureDescriptors& descriptors);
    inline void SetKeypoints(const colmap::FeatureKeypoints& keypoints);
    inline void SetArucoKeypoints(const std::map<int, std::vector<Eigen::Vector2d>>& aruco_keypoints);

    // sift feature descriptors, only used with 3D reconstruction
    inline const colmap::FeatureDescriptors& FeatureDescriptors() const;
    inline colmap::FeatureDescriptors& FeatureDescriptors();
    // corresponding sift feature keypoints, only used with 3D reconstruction
    inline const colmap::FeatureKeypoints& FeatureKeypoints() const;
    inline colmap::FeatureKeypoints& FeatureKeypoints();
    // Map of aruco ids and corresponding point observations, only used with aruco 3D reconstruction
    inline const std::map<int, std::vector<Eigen::Vector2d>>& ArucoKeypoints() const;

   private:
    colmap::FeatureDescriptors feature_descriptors_;
    colmap::FeatureKeypoints feature_keypoints_;
    // this is a ordered map so the position of detected corners is stable regarding their position in the overall map<d
    std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints_;
    std::string name_;
    std::vector<Eigen::Vector2d> points2D_;
    std::unordered_map<size_t, uint32_t> correspondences_;
    Eigen::Quaterniond rotation_;
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

  inline const std::string& Image::Name() const {
    return name_;
  }

  inline void Image::SetName(const std::string& name) {
    name_ = name;
  }

  inline const Eigen::Quaterniond& Image::Rotation() const {
    return rotation_;
  }

  inline Eigen::Quaterniond& Image::Rotation() {
    return rotation_;
  }

  inline void Image::SetRotation(const Eigen::Quaterniond& rotation) {
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

  inline void Image::SetDescriptors(const colmap::FeatureDescriptors& descriptors) {
    feature_descriptors_ = descriptors;
  }

  inline void Image::SetKeypoints(const colmap::FeatureKeypoints& keypoints) {
    feature_keypoints_ = keypoints;
  }

  inline void Image::SetArucoKeypoints(const std::map<int, std::vector<Eigen::Vector2d>>& aruco_keypoints) {
    aruco_keypoints_ = aruco_keypoints;
  }

  inline const colmap::FeatureDescriptors& Image::FeatureDescriptors() const {
    return feature_descriptors_;
  }

  inline colmap::FeatureDescriptors& Image::FeatureDescriptors() {
    return feature_descriptors_;
  }

  inline const colmap::FeatureKeypoints& Image::FeatureKeypoints() const {
    return feature_keypoints_;
  }

  inline colmap::FeatureKeypoints& Image::FeatureKeypoints() {
    return feature_keypoints_;
  }

  inline const std::map<int, std::vector<Eigen::Vector2d>>& Image::ArucoKeypoints() const {
    return aruco_keypoints_;
  }
}