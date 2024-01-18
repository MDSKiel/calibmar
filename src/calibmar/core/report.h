#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "calibmar/extractors/sift_extractor.h"

#include "calibration_targets.h"
#include <optional>
#include <ostream>
#include <variant>

namespace calibmar {
  namespace report {
    // Write human readable calibration report txt to file
    void WriteCalibrationReport(const std::string& path, const Calibration& calibration);
    // Write GEOMAR defined camera information yaml to file
    void WriteCalibrationYaml(const std::string& path, const Calibration& calibration);

    void GenerateCalibrationYaml(std::ostream& stream, const Calibration& calibration);
    std::string GenerateResultString(const Calibration& calibration);
    void GenerateResultString(std::ostream& stream, const Calibration& calibration);

    std::string GenerateCalibrationTargetInfo(const ChessboardFeatureExtractor::Options& options);
    std::string GenerateCalibrationTargetInfo(
        const std::variant<ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options>& options);
  }

  // Represents parameters imported from report files
  class ImportedParameters {
   public:
    int chessboard_rows = 0;
    int chessboard_columns = 0;
    double square_size = 1;
    ArucoMarkerTypes aruco_type = ArucoMarkerTypes::DICT_4X4_50;
    double aruco_scale_factor = 3.0;
    CalibrationTargetType calibration_target = CalibrationTargetType::Chessboard;
    std::string directory;
    CameraModelType camera_model = CameraModelType::SimplePinholeCameraModel;
    std::optional<HousingInterfaceType> housing_model = {};
    std::vector<double> camera_parameters;
    std::vector<double> housing_parameters;

    static ImportedParameters ImportFromYaml(const std::string& path);
    static ImportedParameters ImportFromYaml(std::istream& stream);
  };
}