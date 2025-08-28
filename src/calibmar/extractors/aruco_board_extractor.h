#pragma once

#include "calibmar/core/calibration_targets.h"
#include "extractor.h"

#include <opencv2/aruco.hpp>

namespace calibmar {

  // Extracts corners from a chessboard calibration target image. Also generates corresponding 3D points based on parametrization.
  class ArucoBoardFeatureExtractor : public FeatureExtractor {
   public:
    struct Options {
      // Aruco marker type to detect
      ArucoMarkerTypes aruco_type = ArucoMarkerTypes::DICT_4X4;
      // Origin of the Aruco grid, i.e. where is ID 0
      ArucoGridOrigin grid_origin = ArucoGridOrigin::TopLeft;
      // Direction of the ascending IDs
      ArucoGridDirection grid_direction = ArucoGridDirection::Horizontal;
      // The first id on the board
      int start_id = 0;
      // Aurco grid columns
      int marker_cols = 1;
      // Aurco grid rows
      int marker_rows = 1;
      // length of a marker edge
      double marker_size = 1.0;
      // spacing inbetween markers
      double marker_spacing = 1.0;
      // aruco border bits (i.e. the 'thickness' of the marker border, typically same with as a marker content bit, so 1)
      int border_bits = 1;
    };

    ArucoBoardFeatureExtractor(const Options& options);

    // Extracts marker corners from the Pixmap image and adds them as 2D points to the Image image.
    //
    // @param pixmap Pixmap. Pixmap image which is searched for the configured chessboard calibration target.
    // @param image Image. Image to which the extracted 2D corner points are added, if return is Status::SUCCESS
    //
    // @return Status of the extraction
    Status Extract(Image& image, const Pixmap& pixmap) override;

    // Get target 3D points. Useful to set in the calibration.
    //
    // @return 3D points mapped to their index.
    const std::map<uint32_t, Eigen::Vector3d>& Points3D();

   private:
    Options options_;

    std::map<uint32_t, Eigen::Vector3d> points3D_;
    cv::Ptr<cv::aruco::GridBoard> board_;

    int MapMarkerCornerToPointId(int marker_id, int corner_idx);
  };

  inline const std::map<uint32_t, Eigen::Vector3d>& ArucoBoardFeatureExtractor::Points3D() {
    return points3D_;
  }
}