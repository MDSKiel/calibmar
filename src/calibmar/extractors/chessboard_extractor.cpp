#include "calibmar/extractors/chessboard_extractor.h"

#include "chessboard_extractor.h"
#include <opencv2/calib3d.hpp>

namespace calibmar {
  void ChessboardFeatureExtractor::Options::Check() {
    int limit = 3;
    if (chessboard_rows <= limit || chessboard_columns <= limit) {
      throw std::runtime_error("To few chessboard rows or columns.");
    }
    if (square_size <= 0.0) {
      throw std::runtime_error("Square size must be greater zero.");
    }
  }

  ChessboardFeatureExtractor::ChessboardFeatureExtractor(const Options& options) : options_(options) {
    // one less because openCV will only detect inner corners
    // cv::findChessboardCorners orders "row by row, left to right in every row", so basically row-major.
    uint32_t idx = 0;
    for (int row = 0; row < options_.chessboard_rows - 1; row++) {
      for (int column = 0; column < options_.chessboard_columns - 1; column++) {
        double x = (double)column * options_.square_size;
        double y = (double)row * options_.square_size;
        double z = 0.0;

        points3D_[idx] = {x, y, z};
        idx++;
      }
    }
  }

  FeatureExtractor::Status ChessboardFeatureExtractor::Extract(Image& image, const Pixmap& pixmap) {    
    options_.Check();

    if (pixmap.Width() <= 0 || pixmap.Height() <= 0) {
      return Status::DETECTION_ERROR;
    }

    std::vector<cv::Point2f> corners;
    // opencv expects "inner corners", that is columns-1/rows-1
    cv::Size pattern = cv::Size(options_.chessboard_columns - 1, options_.chessboard_rows - 1);
    bool patternfound = cv::findChessboardCornersSB(pixmap.Data(), pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!patternfound || corners.size() != points3D_.size()) {
      return Status::DETECTION_ERROR;
    }

    std::vector<Eigen::Vector2d> points2D;
    for (const cv::Point2f& corner : corners) {
      points2D.push_back({corner.x, corner.y});
    }

    image.SetPoints2D(points2D);

    // the corner sequence matches the 3D point sequence.
    for (size_t i = 0; i < corners.size(); i++) {
      image.SetPoint3DforPoint2D(i, i);
    }

    return Status::SUCCESS;
  }
}
