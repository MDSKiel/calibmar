#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/readers/image_reader.h"
#include "extractor.h"

namespace calibmar {

  // Extracts corners from a chessboard calibration target image. Also generates corresponding 3D points based on parametrization.
  class ChessboardFeatureExtractor : public FeatureExtractor {
   public:
    struct Options {
      // number of chessboard rows
      int chessboard_rows = 0;
      // number of chessboard columns
      int chessboard_columns = 0;
      // length of a square edge
      double square_size = 1.0;

      void Check();
    };

    ChessboardFeatureExtractor(const Options& options);

    // Extracts inner chessboard corners from the Pixmap image and adds them as 2D points to the Image image.
    //
    // @param pixmap Pixmap. Pixmap image which is searched for the configured chessboard calibration target.
    // @param image Image. Image to which the extracted 2D corner points are added, if return is Status::SUCCESS
    //
    // @return Status of the extraction
    Status Extract(Image& image, const Pixmap& pixmap) override;

    // Get the chessboard target 3D points. Useful to set in the calibration.
    //
    // @return 3D points mapped to their index.
    const std::map<uint32_t, Eigen::Vector3d>& Points3D();

   private:
    Options options_;

    std::map<uint32_t, Eigen::Vector3d> points3D_;
  };

  inline const std::map<uint32_t, Eigen::Vector3d>& ChessboardFeatureExtractor::Points3D() {
    return points3D_;
  }
}