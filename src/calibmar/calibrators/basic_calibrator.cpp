#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/opencv_calibration.h"

#include <algorithm>
#include <colmap/sensor/models.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace calibmar {

  void BasicCalibrator::Options::Check() {
    if (!use_intrinsics_guess && (image_size.first == 0 || image_size.second == 0)) {
      throw std::runtime_error("Image size must be set!.");
    }
  }

  BasicCalibrator::BasicCalibrator(const Options& options) : options_(options) {}

  void BasicCalibrator::Calibrate(Calibration& calibration) {
    options_.Check();

    if (calibration.Images().size() == 0) {
      throw std::runtime_error("No images to calibrate from.");
    }
    if (calibration.Points3D().size() == 0) {
      throw std::runtime_error("3D Points not set.");
    }
    colmap::Camera& camera = calibration.Camera();

    if (options_.use_intrinsics_guess && camera.ModelId() == colmap::kInvalidCameraModelId) {
      throw std::runtime_error("Intrinsics guess specified, but camera not initialized.");
    }

    if (!options_.use_intrinsics_guess) {
      camera.SetWidth(options_.image_size.first);
      camera.SetHeight(options_.image_size.second);
      camera.SetModelIdFromName(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
    }

    std::vector<std::vector<Eigen::Vector2d>> pointSets2D;
    std::vector<std::vector<Eigen::Vector3d>> pointSets3D;
    calibration.GetCorrespondences(pointSets2D, pointSets3D);

    std::vector<Eigen::Quaterniond*> rotation_vecs;
    std::vector<Eigen::Vector3d*> translation_vecs;
    for (auto& image : calibration.Images()) {
      rotation_vecs.push_back(&image.Rotation());
      translation_vecs.push_back(&image.Translation());
    }

    std::vector<double> std_deviations_intrinsics, std_deviations_extrinsics, per_view_rms;
    double rms = opencv_calibration::CalibrateCamera(pointSets3D, pointSets2D, camera, options_.use_intrinsics_guess,
                                                     options_.fast, rotation_vecs, translation_vecs, std_deviations_intrinsics,
                                                     std_deviations_extrinsics, per_view_rms);

    calibration.SetCalibrationRms(rms);
    calibration.SetPerViewRms(per_view_rms);
    std_deviations_intrinsics.erase(std::remove(std_deviations_intrinsics.begin(), std_deviations_intrinsics.end(), 0.0),
                                    std_deviations_intrinsics.end());
    calibration.SetIntrinsicsStdDeviations(std_deviations_intrinsics);
  }
}