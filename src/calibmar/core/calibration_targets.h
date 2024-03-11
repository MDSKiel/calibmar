#pragma once

#include <opencv2/aruco.hpp>

namespace calibmar {

  enum class CalibrationTargetType { Chessboard, ArucoGridBoard, Target3D, Target3DAruco };

  enum class ArucoMarkerTypes {
    DICT_4X4_50 = cv::aruco::DICT_4X4_50,
    DICT_4X4_100 = cv::aruco::DICT_4X4_100,
    DICT_4X4_250 = cv::aruco::DICT_4X4_250,
    DICT_4X4_1000 = cv::aruco::DICT_4X4_1000,
    DICT_5X5_50 = cv::aruco::DICT_5X5_50,
    DICT_5X5_100 = cv::aruco::DICT_5X5_100,
    DICT_5X5_250 = cv::aruco::DICT_5X5_250,
    DICT_5X5_1000 = cv::aruco::DICT_5X5_1000,
    DICT_6X6_50 = cv::aruco::DICT_6X6_50,
    DICT_6X6_100 = cv::aruco::DICT_6X6_100,
    DICT_6X6_250 = cv::aruco::DICT_6X6_250,
    DICT_6X6_1000 = cv::aruco::DICT_6X6_1000,
    DICT_7X7_50 = cv::aruco::DICT_7X7_50,
    DICT_7X7_100 = cv::aruco::DICT_7X7_100,
    DICT_7X7_250 = cv::aruco::DICT_7X7_250,
    DICT_7X7_1000 = cv::aruco::DICT_7X7_1000,
    DICT_ARUCO_ORIGINAL = cv::aruco::DICT_ARUCO_ORIGINAL,
    DICT_APRILTAG_16h5 = cv::aruco::DICT_APRILTAG_16h5,
    DICT_APRILTAG_25h9 = cv::aruco::DICT_APRILTAG_25h9,
    DICT_APRILTAG_36h10 = cv::aruco::DICT_APRILTAG_36h10,
    DICT_APRILTAG_36h11 = cv::aruco::DICT_APRILTAG_36h11
  };

  enum class ArucoGridOrigin { TopLeft, TopRight, BottomLeft, BottomRight };

  enum class ArucoGridDirection { Horizontal, Vertical };

  namespace calibration_targets {
    inline std::vector<std::pair<std::string, ArucoMarkerTypes>> ArucoTypes() {
      // These names are used in serialization, and can not be changed without breaking compatibility with old reports
      return {{"4X4_50", ArucoMarkerTypes::DICT_4X4_50},
              {"4X4_100", ArucoMarkerTypes::DICT_4X4_100},
              {"4X4_250", ArucoMarkerTypes::DICT_4X4_250},
              {"4X4_1000", ArucoMarkerTypes::DICT_4X4_1000},
              {"5X5_50", ArucoMarkerTypes::DICT_5X5_50},
              {"5X5_100", ArucoMarkerTypes::DICT_5X5_100},
              {"5X5_250", ArucoMarkerTypes::DICT_5X5_250},
              {"5X5_1000", ArucoMarkerTypes::DICT_5X5_1000},
              {"6X6_50", ArucoMarkerTypes::DICT_6X6_50},
              {"6X6_100", ArucoMarkerTypes::DICT_6X6_100},
              {"6X6_250", ArucoMarkerTypes::DICT_6X6_250},
              {"6X6_1000", ArucoMarkerTypes::DICT_6X6_1000},
              {"7X7_50", ArucoMarkerTypes::DICT_7X7_50},
              {"7X7_100", ArucoMarkerTypes::DICT_7X7_100},
              {"7X7_250", ArucoMarkerTypes::DICT_7X7_250},
              {"7X7_1000", ArucoMarkerTypes::DICT_7X7_1000},
              {"ARUCO_ORIGINAL", ArucoMarkerTypes::DICT_ARUCO_ORIGINAL},
              {"APRILTAG_16h5", ArucoMarkerTypes::DICT_APRILTAG_16h5},
              {"APRILTAG_25h9", ArucoMarkerTypes::DICT_APRILTAG_25h9},
              {"APRILTAG_36h10", ArucoMarkerTypes::DICT_APRILTAG_36h10},
              {"APRILTAG_36h11", ArucoMarkerTypes::DICT_APRILTAG_36h11}};
    }

    inline std::string NameFromArucoType(ArucoMarkerTypes type) {
      for (const auto& name_type : ArucoTypes()) {
        if (name_type.second == type) {
          return name_type.first;
        }
      }

      throw std::runtime_error("Unkown marker type!");
    }

    inline ArucoMarkerTypes ArucoTypeFromName(const std::string& name) {
      for (const auto& name_type : ArucoTypes()) {
        if (name_type.first == name) {
          return name_type.second;
        }
      }

      throw std::runtime_error("Unkown marker name!");
    }
  }
}