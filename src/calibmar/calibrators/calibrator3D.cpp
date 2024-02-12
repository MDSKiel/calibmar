#include "calibrator3D.h"

#include "calibmar/calibrators/general_calibration.h"
#include "calibmar/core/camera_models.h"

#include "colmap/controllers/automatic_reconstruction.h"
#include "colmap/controllers/feature_matching.h"
#include "colmap/controllers/incremental_mapper.h"

#include "colmap_calibration.h"

#include <sstream>

namespace {
  void SetupArucoCornerData(const calibmar::Calibration& calibration, std::vector<std::vector<Eigen::Vector3d>>& object_points,
                            std::vector<std::vector<Eigen::Vector2d>>& image_points) {
    // Each image contains one or more aruco markers. Each marker has four ordered corners.
    // All markers are considered to be of the same physical size and therefore interchangable
    std::vector<Eigen::Vector3d> aruco_points_3D{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    object_points.reserve(calibration.Images().size());
    image_points.reserve(calibration.Images().size());

    for (const calibmar::Image& image : calibration.Images()) {
      for (const auto& id_points : image.ArucoKeypoints()) {
        object_points.push_back(aruco_points_3D);
        image_points.push_back(id_points.second);
      }
    }
  }
}

namespace calibmar {

  Calibrator3D::Calibrator3D(const Options& options) : options_(options) {}

  void Calibrator3D::Calibrate(Calibration& calibration) {
    if (calibration.Images().size() == 0) {
      throw std::runtime_error("No images to calibrate from.");
    }

    if (options_.enable_refraction) {
      if (!options_.use_intrinsics_guess) {
        throw std::runtime_error("Housing calibration requires an intrinsic guess!");
      }
      if (!calibration.Camera().IsCameraRefractive()) {
        throw std::runtime_error("Housing calibration requested but refractive parameters not initialized!");
      }
    }

    if (!options_.use_intrinsics_guess) {
      if ((options_.image_size.first == 0 || options_.image_size.second == 0)) {
        throw std::runtime_error("Image size must be set!.");
      }

      // for colmap reconstruction camera parameters must always be set to some relatively sane value
      double focal_length = 1.2 * std::max(options_.image_size.first, options_.image_size.second);
      calibration.SetCamera(colmap::Camera::CreateFromModelName(
          colmap::kInvalidCameraId, CameraModel::CameraModels().at(options_.camera_model).model_name, focal_length,
          options_.image_size.first, options_.image_size.second));

      if (options_.contains_arucos) {
        // if arucos were used and no intrinsics given, the aruco corners are used in a first 2D-3D calibration from aruco
        // corners.
        std::vector<std::vector<Eigen::Vector3d>> object_points;
        std::vector<std::vector<Eigen::Vector2d>> image_points;
        SetupArucoCornerData(calibration, object_points, image_points);

        // do a first calibration using aruco marker edges to get a good first estimate
        // estimate always on pinhole. The assumption is, that the aruco marker corners are always relatively close together in
        // the image where the distortion will not have a big effect on the 2D obsevations (and therefore also be ill suited to
        // determine distortion).
        colmap::Camera init_camera = colmap::Camera::CreateFromModelName(
            colmap::kInvalidCameraId, CameraModel::CameraModels().at(CameraModelType::SimplePinholeCameraModel).model_name,
            focal_length, options_.image_size.first, options_.image_size.second);
        std::vector<colmap::Rigid3d> poses;
        general_calibration::CalibrateCamera(object_points, image_points, init_camera, false, poses);

        std::cout << "Initial camera estimate from arucos:" << std::endl << init_camera.CalibrationMatrix() << std::endl;

        if (calibration.Camera().FocalLengthIdxs().size() == 1) {
          calibration.Camera().SetFocalLength(init_camera.FocalLength());
        }
        else {
          calibration.Camera().SetFocalLengthX(init_camera.FocalLength());
          calibration.Camera().SetFocalLengthY(init_camera.FocalLength());
        }
      }
    }

    colmap_calibration::CalibrateCamera(calibration, reconstruction_, options_.enable_refraction);
  }
}
