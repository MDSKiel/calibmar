#include "housing_calibrator.h"
#include "general_calibration.h"
#include "non_svp_calibration.h"

namespace calibmar {
  void HousingCalibrator::Options::Check() {
    int number_params_camera = calibmar::CameraModel::CameraModels().at(camera_model).num_params;
    if (camera_params.size() != number_params_camera) {
      throw std::runtime_error("Number of camera parameters does not match camera model.");
    }
    int number_params_housing = calibmar::HousingInterface::HousingInterfaces().at(housing_interface).num_params;
    if (initial_housing_params.size() != number_params_housing) {
      throw std::runtime_error("Number of initial housing parameters does not match housing type.");
    }
    if (estimate_initial_dome_offset && (pattern_cols_rows.first <= 0 || pattern_cols_rows.second <= 0)) {
      throw std::runtime_error("Pattern size needs to be set when estimating initial dome offset.");
    }
    if (image_size.first <= 0 || image_size.second <= 0) {
      throw std::runtime_error("Image size must be set!.");
    }
  }

  HousingCalibrator::HousingCalibrator(const Options& options) : options_(options) {}

  void HousingCalibrator::Calibrate(Calibration& calibration) {
    options_.Check();

    if (calibration.Images().size() == 0) {
      throw std::runtime_error("No images to calibrate from.");
    }
    if (calibration.Points3D().size() == 0) {
      throw std::runtime_error("3D Points not set.");
    }

    colmap::Camera& camera = calibration.Camera();
    camera.width = options_.image_size.first;
    camera.height = options_.image_size.second;
    camera.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(options_.camera_model).model_name);
    camera.refrac_model_id = colmap::CameraRefracModelNameToId(
        calibmar::HousingInterface::HousingInterfaces().at(options_.housing_interface).model_name);

    std::vector<std::vector<Eigen::Vector2d>> point_sets_2D;
    std::vector<std::vector<Eigen::Vector3d>> point_sets_3D;
    calibration.GetCorrespondences(point_sets_2D, point_sets_3D);
    std::vector<Image>& images = calibration.Images();

    camera.params = options_.camera_params;
    camera.refrac_params = options_.initial_housing_params;

    if (camera.refrac_model_id == colmap::FlatPort::refrac_model_id) {
      Eigen::Map<Eigen::Vector3d> normal(camera.refrac_params.data());
      if (abs(normal.norm() - 1.0) > 1e-6) {
        throw std::runtime_error("Interface normal must be normalized to unit length!");
      }

      // normalize explicitly again
      normal.normalize();
    }

    if (options_.estimate_initial_dome_offset && camera.refrac_model_id == colmap::DomePort::refrac_model_id) {
      non_svp_calibration::EstimateInitialDomeOffset(
          point_sets_3D[0], point_sets_2D[0],
          {options_.pattern_cols_rows.first - 1,
           options_.pattern_cols_rows.second - 1},  // the expected pattern size here matches opencv conventions
          camera);
    }
    
    for (size_t i = 0; i < images.size(); i++) {
      Image& image = images[i];
      std::vector<Eigen::Vector2d>& points2D = point_sets_2D[i];
      std::vector<Eigen::Vector3d>& points3D = point_sets_3D[i];

      non_svp_calibration::EstimateAbsolutePoseRefractiveCamera(points3D, points2D, &image.Rotation(), &image.Translation(),
                                                                &camera, false);
    }

    // std::vector<double>& housing_params_std = calibration.HousingParamsStdDeviations();
    // non_svp_calibration::OptimizeRefractiveCamera(calibration, housing_params_std);

    general_calibration::OptimizeCamera(calibration);

    std::vector<colmap::Rigid3d> poses;
    for (const auto& image : calibration.Images()) {
      poses.reserve(images.size());
      poses.push_back(colmap::Rigid3d(image.Rotation(), image.Translation()));
    }

    std::vector<double> per_view_rms;
    double rms =
        general_calibration::CalculateOverallRMS(point_sets_3D, point_sets_2D, poses, calibration.Camera(), per_view_rms);

    calibration.SetCalibrationRms(rms);
    calibration.SetPerViewRms(per_view_rms);
  }
}