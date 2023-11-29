#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/extractor.h"
#include "calibrator.h"

#include <colmap/scene/database.h>
#include <colmap/scene/reconstruction.h>

#include <optional>

namespace calibmar {
  class Calibrator3D : public Calibrator {
   public:
    struct Options {
      CameraModelType camera_model = CameraModelType::OpenCVCameraModel;
      // If true, the calibration camera model will be used as an initial intrinsics guess as is.
      // Will also deactivate aruco based instrinsics precalibration
      bool use_intrinsics_guess = false;

      bool contains_arucos = false;

      std::pair<int, int> image_size = {0, 0};
    };

    Calibrator3D(const Options& options);

    void Calibrate(Calibration& calibration) override;

    std::shared_ptr<colmap::Reconstruction> Reconstruction();

   private:
    Options options_;
    std::shared_ptr<colmap::Reconstruction> reconstruction_;
  };

  inline std::shared_ptr<colmap::Reconstruction> Calibrator3D::Reconstruction() {
    return reconstruction_;
  }
}