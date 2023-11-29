#pragma once

#include <opencv2/aruco.hpp>

namespace calibmar {

  enum class CalibrationTargetType { Chessboard, Target3D, Target3DAruco };

  namespace calibration_targets {

    inline std::vector<std::pair<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>> ArucoTypes() {
      return {{"4X4_50", cv::aruco::DICT_4X4_50},
              {"4X4_100", cv::aruco::DICT_4X4_100},
              {"4X4_250", cv::aruco::DICT_4X4_250},
              {"4X4_1000", cv::aruco::DICT_4X4_1000},
              {"5X5_50", cv::aruco::DICT_5X5_50},
              {"5X5_100", cv::aruco::DICT_5X5_100},
              {"5X5_250", cv::aruco::DICT_5X5_250},
              {"5X5_1000", cv::aruco::DICT_5X5_1000},
              {"6X6_50", cv::aruco::DICT_6X6_50},
              {"6X6_100", cv::aruco::DICT_6X6_100},
              {"6X6_250", cv::aruco::DICT_6X6_250},
              {"6X6_1000", cv::aruco::DICT_6X6_1000},
              {"7X7_50", cv::aruco::DICT_7X7_50},
              {"7X7_100", cv::aruco::DICT_7X7_100},
              {"7X7_250", cv::aruco::DICT_7X7_250},
              {"7X7_1000", cv::aruco::DICT_7X7_1000},
              {"ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
              {"APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
              {"APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
              {"APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
              {"APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}};
    }
  }
}