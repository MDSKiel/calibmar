#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/aruco_board_extractor.h"
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
    std::string GenerateCalibrationTargetInfo(const ArucoBoardFeatureExtractor::Options& options);
    std::string GenerateCalibrationTargetInfo(
        const std::variant<ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options>& options);
  }

  // Represents parameters imported from report files
  class ImportedParameters {
   public:
    // Rows of chess or aruco board
    int rows = 0;
    // Columns of chess or aruco board
    int columns = 0;
    // Square/marker size of chess/aruco board
    double square_size = 1;
    // Spacing between aruco board markers
    double spacing = 0.01;
    // Aruco marker type of auto-calibration target or aruco board
    ArucoMarkerTypes aruco_type = ArucoMarkerTypes::DICT_4X4_50;
    // Origin of the markers for a aruco grid board (i.e. where on the board is marker id 0)
    ArucoGridOrigin grid_origin = ArucoGridOrigin::TopLeft;
    // Direction of ascending ids for a aruco grid board
    ArucoGridDirection grid_direction = ArucoGridDirection::Horizontal;
    // Number of bits in the aruco marker border (i.e. size of the black marker border)
    int border_bits = 1;
    // Scale factor to create the SIFT feature mask from aruco marker contour
    double aruco_scale_factor = 3.0;
    // Type of calibration target
    CalibrationTargetType calibration_target = CalibrationTargetType::Chessboard;
    // Directory of the imported parameters/calibration
    std::string directory;
    // Type of camera model used
    CameraModelType camera_model = CameraModelType::SimplePinholeCameraModel;
    // If housing calibration, the type of housing model used
    std::optional<HousingInterfaceType> housing_model = {};
    // camera parameters
    std::vector<double> camera_parameters;
    // housing parameters (only if housing calibration)
    std::vector<double> housing_parameters;

    static ImportedParameters ImportFromYaml(const std::string& path);
    static ImportedParameters ImportFromYaml(std::istream& stream);
  };
}