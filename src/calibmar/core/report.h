#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"

#include <optional>
#include <ostream>

namespace calibmar {
  namespace report {
    // Write human readable calibration report txt to file
    void WriteCalibrationReport(const std::string& path, const Calibration& calibration);
    // Write GEOMAR defined camera information yaml to file
    void WriteCalibrationYaml(const std::string& path, const Calibration& calibration);

    void GenerateCalibrationYaml(std::ostream& stream, const Calibration& calibration);
    std::string GenerateResultString(const Calibration& calibration);
    void GenerateResultString(std::ostream& stream, const Calibration& calibration);
  }

  // Represents parameters imported from report files
  class ImportedParameters {
   public:
    int chessboard_rows = 0;
    int chessboard_columns = 0;
    double square_size = 1;
    std::string directory;
    CameraModelType camera_model = CameraModelType::SimplePinholeCameraModel;
    std::optional<HousingInterfaceType> housing_model = {};
    std::vector<double> camera_parameters;
    std::vector<double> housing_parameters;

    static ImportedParameters ImportFromYaml(const std::string& path);
    static ImportedParameters ImportFromYaml(std::istream& stream);
  };
}