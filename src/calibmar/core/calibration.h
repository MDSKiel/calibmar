#pragma once

#include "calibmar/core/image.h"

#include <colmap/src/base/camera.h>
#include <unordered_map>

namespace calibmar {
  // Calibration holds all information about a single calibration.
  class Calibration {
   public:
    colmap::Camera& Camera();
    const colmap::Camera& Camera() const;
    void SetCamera(const colmap::Camera& camera);

    class Image& Image(size_t image_idx);
    const class Image& Image(size_t image_idx) const;
    size_t AddImage(const class Image& image);

    Eigen::Vector3d& Point3D(uint32_t point_id);
    const Eigen::Vector3d& Point3D(uint32_t point_id) const;
    uint32_t AddPoint3D(const Eigen::Vector3d& xyz);

    std::unordered_map<uint32_t, Eigen::Vector3d>& Points3D();
    const std::unordered_map<uint32_t, Eigen::Vector3d>& Points3D() const;
    void SetPoints3D(const std::unordered_map<uint32_t, Eigen::Vector3d>& points3D);

    std::vector<class Image>& Images();
    const std::vector<class Image>& Images() const;

    double CalibrationRms() const;
    void SetCalibrationRms(double rms);

    std::vector<double>& IntrinsicsStdDeviations();
    const std::vector<double>& IntrinsicsStdDeviations() const;
    void SetIntrinsicsStdDeviations(const std::vector<double>& intrinsincs_std);

    std::vector<double>& HousingParamsStdDeviations();
    const std::vector<double>& HousingParamsStdDeviations() const;
    void SetHousingParamsStdDeviations(const std::vector<double>& housing_std);

    // Set infos about the used calibration target. Used in report generation.
    void SetCalibrationTargetInfo(const std::string& info);
    // Get infos about the used calibration target. Used in report generation.
    const std::string& GetCalibrationTargetInfo() const;

    void GetCorrespondences(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                            std::vector<std::vector<Eigen::Vector3d>>& points3D);

   private:
    colmap::Camera camera_;
    double calibration_rms_ = 0;
    std::string calibration_target_info_;
    std::vector<double> intrinsics_std_deviations_;
    std::vector<double> housing_params_std_deviations_;
    std::vector<class Image> images_;
    std::unordered_map<uint32_t, Eigen::Vector3d> points3D_;
    // used to generate point id
    uint32_t number_of_points3D = 0;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////

  inline colmap::Camera& Calibration::Camera() {
    return camera_;
  }
  inline const colmap::Camera& Calibration::Camera() const {
    return camera_;
  }

  inline void Calibration::SetCamera(const colmap::Camera& camera) {
    camera_ = camera;
  }

  inline class Image& Calibration::Image(const size_t image_idx) {
    return images_.at(image_idx);
  }
  inline const class Image& Calibration::Image(const size_t image_idx) const {
    return images_.at(image_idx);
  }

  inline size_t Calibration::AddImage(const class Image& image) {
    images_.push_back(image);
    return images_.size() - 1;
  }

  inline Eigen::Vector3d& Calibration::Point3D(uint32_t point_id) {
    return points3D_.at(point_id);
  }
  inline const Eigen::Vector3d& Calibration::Point3D(uint32_t point_id) const {
    return points3D_.at(point_id);
  }

  inline uint32_t Calibration::AddPoint3D(const Eigen::Vector3d& xyz) {
    uint32_t id = number_of_points3D;
    points3D_[number_of_points3D] = xyz;
    number_of_points3D++;
    return id;
  }

  inline std::unordered_map<uint32_t, Eigen::Vector3d>& Calibration::Points3D() {
    return points3D_;
  }
  inline const std::unordered_map<uint32_t, Eigen::Vector3d>& Calibration::Points3D() const {
    return points3D_;
  }

  inline void Calibration::SetPoints3D(const std::unordered_map<uint32_t, Eigen::Vector3d>& points3D) {
    points3D_ = points3D;
  }

  inline std::vector<class Image>& Calibration::Images() {
    return images_;
  }
  inline const std::vector<class Image>& Calibration::Images() const {
    return images_;
  }

  inline double Calibration::CalibrationRms() const {
    return calibration_rms_;
  }

  inline void Calibration::SetCalibrationRms(double rms) {
    calibration_rms_ = rms;
  }

  inline std::vector<double>& Calibration::IntrinsicsStdDeviations() {
    return intrinsics_std_deviations_;
  }
  inline const std::vector<double>& Calibration::IntrinsicsStdDeviations() const {
    return intrinsics_std_deviations_;
  }

  inline void Calibration::SetIntrinsicsStdDeviations(const std::vector<double>& intrinsincs_rms) {
    intrinsics_std_deviations_ = intrinsincs_rms;
  }

  inline std::vector<double>& Calibration::HousingParamsStdDeviations() {
    return housing_params_std_deviations_;
  }
  inline const std::vector<double>& Calibration::HousingParamsStdDeviations() const {
    return housing_params_std_deviations_;
  }

  inline void Calibration::SetHousingParamsStdDeviations(const std::vector<double>& housing_std) {
    housing_params_std_deviations_ = housing_std;
  }

  inline void Calibration::SetCalibrationTargetInfo(const std::string& info) {
    calibration_target_info_ = info;
  }

  inline const std::string& Calibration::GetCalibrationTargetInfo() const {
    return calibration_target_info_;
  }
}