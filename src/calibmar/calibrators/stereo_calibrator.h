#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/extractor.h"
#include "calibrator.h"

#include <optional>

namespace calibmar {
  class StereoCalibrator {
   public:
    struct Options {
      CameraModelType camera_model = CameraModelType::OpenCVCameraModel;
      // If true, the calibration camera model will be used as an initial intrinsics guess as is.
      bool use_intrinsics_guess = false;
      // Image size in pixel (width, height)
      std::pair<int, int> image_size = {0, 0};

      void Check();
    };

    StereoCalibrator(const Options& options);

    void Calibrate(Calibration& calibration1, Calibration& calibration2);

   private:
    Options options_;
  };
}