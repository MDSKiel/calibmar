#include "calibmar/calibrators/stereo_calibrator.h"
#include "calibmar/calibrators/general_calibration.h"
#include "calibmar/calibrators/stereo_calibration.h"

#include <colmap/sensor/models.h>

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
        (camera1.model_id == colmap::CameraModelId::kInvalid || camera2.model_id == colmap::CameraModelId::kInvalid)) {
      throw std::runtime_error("Intrinsics guess specified, but camera not initialized.");
    }

    if (!options_.use_intrinsics_guess) {
      camera1.width = options_.image_size.first;
      camera1.height = options_.image_size.second;
      camera1.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
      camera2.width = options_.image_size.first;
      camera2.height = options_.image_size.second;
      camera2.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
    }

    std::vector<std::vector<Eigen::Vector2d>> pointSets2D_1, pointSets2D_2;
    std::vector<std::vector<Eigen::Vector3d>> pointSets3D_1, pointSets3D_2;
    calibration1.GetCorrespondences(pointSets2D_1, pointSets3D_1);
    calibration2.GetCorrespondences(pointSets2D_2, pointSets3D_2);

    std::vector<std::vector<double>> per_view_rms(2);
    colmap::Rigid3d relative_pose;
    std::vector<colmap::Rigid3d> poses;

    std::vector<double> std_dev_camera1;
    std::vector<double> std_dev_camera2;
    stereo_calibration::StereoStdDeviations std_devs;
    std_devs.std_deviations_intrinsics1 = &std_dev_camera1;
    std_devs.std_deviations_intrinsics2 = &std_dev_camera2;
    // extrinsics std devs are currently not used, because their interpretation is unclear

    stereo_calibration::CalibrateStereoCameras(pointSets3D_1, pointSets2D_1, pointSets2D_2, camera1, camera2,
                                               options_.use_intrinsics_guess, options_.estimate_pose_only, relative_pose, poses,
                                               &std_devs);

    std::vector<colmap::Rigid3d> poses2;
    poses2.reserve(poses.size());
    for (const auto& pose1 : poses) {
      // calculate poses of the second camera
      poses2.push_back(relative_pose * pose1);
    }

    double rms1 = general_calibration::CalculateOverallRMS(pointSets3D_1, pointSets2D_1, poses, camera1, per_view_rms[0]);
    double rms2 = general_calibration::CalculateOverallRMS(pointSets3D_1, pointSets2D_2, poses2, camera2, per_view_rms[1]);

    calibration1.SetCalibrationRms(rms1);
    calibration1.SetPerViewRms(per_view_rms[0]);
    calibration1.SetIntrinsicsStdDeviations(*std_devs.std_deviations_intrinsics1);
    calibration2.SetCalibrationRms(rms2);
    calibration2.SetPerViewRms(per_view_rms[1]);
    calibration2.SetIntrinsicsStdDeviations(*std_devs.std_deviations_intrinsics2);

    // The calibration pose is defined as camera to world and camera 1 is supposed to be world here
    // The pose from StereoCalibrateCamera() is camera1 to camera2, so we need to invert it here (to get 2 to 1, i.e. 2 to world).
    calibration1.SetCameraToWorldStereo(colmap::Rigid3d());
    calibration2.SetCameraToWorldStereo(colmap::Inverse(relative_pose));
  }
}