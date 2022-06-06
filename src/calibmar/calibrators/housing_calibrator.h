#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/extractor.h"

#include <optional>

namespace calibmar {
  // HousingCalibrator class calibrates the housing using contained 2D-3D correspondences.
  class HousingCalibrator {
   public:
    struct Options {
      CameraModelType camera_model = CameraModelType::OpenCVCameraModel;
      HousingInterfaceType housing_interface = HousingInterfaceType::DoubleLayerSphericalRefractive;
      // Required camera parameters. Must match camera_model.
      std::vector<double> camera_params;
      // Required initial housing params. Must match housing_interface. The first three parameters (the offsets)
      // will be optimized and can be 0. The others containing interface constants must be set.
      std::vector<double> initial_housing_params;
      // Optionally disable estimating the intial dome offset before the optimization step.
      // (E.g. if the parameters contain a valid guess).
      bool estimate_initial_dome_offset = true;
      // Columns and rows of the chessboard target.
      std::pair<int, int> pattern_cols_rows = {0, 0};
      // Image size in pixel (width, height).
      std::pair<int, int> image_size = {0, 0};

      void Check();
    };

    HousingCalibrator(const Options& options);

    // Calibrates non svp parameters using the 3D points and images.
    // The calibration camera non svp parameters, image poses, estimated parameters standard deviations
    // and reprojection error will be set.
    //
    // @param calibration Calibration. Must contain 3D Points and images containing corresponding 2D points.
    void Calibrate(Calibration& calibration);

   private:
    Options options_;
  };
}