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

    inline size_t AddPoint2D(const Eigen::Vector2d& point);
    inline void SetPoints2D(const std::vector<Eigen::Vector2d>& points);
    inline const std::vector<Eigen::Vector2d>& Points2D() const;
    inline Eigen::Vector2d Point2D(size_t idx) const;

    inline void SetPoint3DforPoint2D(const uint32_t point3D_id, const size_t point2D_idx);
    // Correspondences of image 2D point indices to calibration 3D point ids
    inline const std::unordered_map<size_t, uint32_t>& Correspondences() const;
    inline void ClearCorrespondences();

    // Pose which is defined as the transformation from world to camera space.
    inline const colmap::Rigid3d& Pose() const;
    inline colmap::Rigid3d& Pose();
    inline void SetPose(const colmap::Rigid3d& pose);

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
    // this is a ordered map so the position of detected corners is stable regarding their position in the overall map
    std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints_;
    std::string name_;
    std::vector<Eigen::Vector2d> points2D_;
    std::unordered_map<size_t, uint32_t> correspondences_;
    colmap::Rigid3d pose_;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////

  inline size_t Image::AddPoint2D(const Eigen::Vector2d& point) {
    points2D_.push_back(point);
    return points2D_.size() - 1;
  }

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

  inline void Image::ClearCorrespondences() {
    correspondences_.clear();
  }

  inline const std::string& Image::Name() const {
    return name_;
  }

  inline void Image::SetName(const std::string& name) {
    name_ = name;
  }

  inline const colmap::Rigid3d& Image::Pose() const {
    return pose_;
  }

  inline colmap::Rigid3d& Image::Pose() {
    return pose_;
  }

  inline void Image::SetPose(const colmap::Rigid3d& pose) {
    pose_ = pose;
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