#include "calibmar/calibrators/stereo_calibrator.h"
#include "calibmar/calibrators/opencv_calibration.h"

#include <algorithm>
#include <colmap/sensor/models.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace calibmar {

  void StereoCalibrator::Options::Check() {
    if (!use_intrinsics_guess && (image_size.first == 0 || image_size.second == 0)) {
      throw std::runtime_error("Image size must be set!.");
    }
  }

  StereoCalibrator::StereoCalibrator(const Options& options) : options_(options) {}

  void StereoCalibrator::Calibrate(Calibration& calibration1, Calibration& calibration2) {
    options_.Check();

    if (calibration1.Images().size() == 0) {
      throw std::runtime_error("No images to calibrate from.");
    }
    if (calibration1.Images().size() != calibration2.Images().size()) {
      throw std::runtime_error("Stereo calibration images are not balanced.");
    }

    if (calibration1.Points3D().size() == 0) {
      throw std::runtime_error("3D Points not set.");
    }

    colmap::Camera& camera1 = calibration1.Camera();
    colmap::Camera& camera2 = calibration2.Camera();
    if (options_.use_intrinsics_guess &&
        (camera1.ModelId() == colmap::kInvalidCameraModelId || camera2.ModelId() == colmap::kInvalidCameraId)) {
      throw std::runtime_error("Intrinsics guess specified, but camera not initialized.");
    }

    if (!options_.use_intrinsics_guess) {
      camera1.SetWidth(options_.image_size.first);
      camera1.SetHeight(options_.image_size.second);
      camera1.SetModelIdFromName(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
      camera2.SetWidth(options_.image_size.first);
      camera2.SetHeight(options_.image_size.second);
      camera2.SetModelIdFromName(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
    }

    std::vector<std::vector<Eigen::Vector2d>> pointSets2D_1, pointSets2D_2;
    std::vector<std::vector<Eigen::Vector3d>> pointSets3D_1, pointSets3D_2;
    calibration1.GetCorrespondences(pointSets2D_1, pointSets3D_1);
    calibration2.GetCorrespondences(pointSets2D_2, pointSets3D_2);

    std::vector<std::vector<double>> per_view_rms;
    colmap::Rigid3d pose;
    double rms = opencv_calibration::StereoCalibrateCamera(pointSets3D_1, pointSets2D_1, pointSets2D_2, camera1, camera2, pose,
                                                           options_.use_intrinsics_guess, false, false, per_view_rms);

    calibration1.SetCalibrationRms(rms);
    calibration1.SetPerViewRms(per_view_rms[0]);
    calibration2.SetCalibrationRms(rms);
    calibration2.SetPerViewRms(per_view_rms[1]);

    // The calibration pose is defined as camera to world and camera 1 is supposed to be world here
    // The pose from StereoCalibrateCamera() is camera1 to camera2, so we need to invert it here (to get 2 to 1, i.e. 2 to world).
    calibration1.SetCameraToWorldStereo(colmap::Rigid3d());
    calibration2.SetCameraToWorldStereo(colmap::Inverse(pose));
  }
}