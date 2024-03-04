#include "calibration_target_visualization.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace {
  void ScaleMarkerCornersInPlace(std::vector<Eigen::Vector2d>& marker, double scale_factor) {
    double mean_x = 0;
    double mean_y = 0;

    for (const auto& corner : marker) {
      mean_x += corner.x();
      mean_y += corner.y();
    }

    mean_x /= marker.size();
    mean_y /= marker.size();

    // to scale round center, move mean to origin, scale and then move back to original location
    for (auto& corner : marker) {
      corner.x() = (corner.x() - mean_x) * scale_factor + mean_x;
      corner.y() = (corner.y() - mean_y) * scale_factor + mean_y;
    }
  }

  void DrawMarker(calibmar::Pixmap& image, const std::vector<Eigen::Vector2d>& corners) {
    cv::line(image.Data(), cv::Point(corners[0].x(), corners[0].y()), cv::Point(corners[1].x(), corners[1].y()),
             cv::Scalar(0, 0, 255));
    cv::line(image.Data(), cv::Point(corners[1].x(), corners[1].y()), cv::Point(corners[2].x(), corners[2].y()),
             cv::Scalar(0, 0, 255));
    cv::line(image.Data(), cv::Point(corners[2].x(), corners[2].y()), cv::Point(corners[3].x(), corners[3].y()),
             cv::Scalar(0, 0, 255));
    cv::line(image.Data(), cv::Point(corners[3].x(), corners[3].y()), cv::Point(corners[0].x(), corners[0].y()),
             cv::Scalar(0, 0, 255));
  }
}

namespace calibmar {

  ChessboardTargetVisualizer::ChessboardTargetVisualizer(int cols, int rows) : cols_(cols), rows_(rows) {}

  void ChessboardTargetVisualizer::DrawTargetOnImage(Pixmap& image, const Image& image_data) const {
    std::vector<cv::Point2f> cornerPoints;
    for (const Eigen::Vector2d& point : image_data.Points2D()) {
      cornerPoints.push_back(cv::Point2f(point.x(), point.y()));
    }
    cv::drawChessboardCorners(image.Data(), cv::Size(cols_ - 1, rows_ - 1), cornerPoints, true);
  }

  Target3DTargetVisualizer::Target3DTargetVisualizer(bool contains_aruco, double scale_factor)
      : contains_aruco_(contains_aruco), scale_factor_(scale_factor) {}

  void Target3DTargetVisualizer::DrawTargetOnImage(Pixmap& image, const Image& image_data) const {
    if (contains_aruco_ && image_data.ArucoKeypoints().size() > 0) {
      for (const auto& id_corners : image_data.ArucoKeypoints()) {
        std::vector<Eigen::Vector2d> corners = id_corners.second;
        DrawMarker(image, corners);
        cv::circle(image.Data(), cv::Point(corners[0].x(), corners[0].y()), 2, cv::Scalar(0, 255, 0));
        ScaleMarkerCornersInPlace(corners, scale_factor_);
        DrawMarker(image, corners);
      }
    }

    for (auto& point2d : image_data.Points2D()) {
      cv::circle(image.Data(), cv::Point(point2d.x(), point2d.y()), 2, cv::Scalar(255, 0, 0));
    }
  }

  void ChessboardTargetVisualizer::DrawTargetPoseOnImage(Pixmap& image,
                                                         const std::vector<Eigen::Vector2d>& target_pose_points) const {
    if (target_pose_points.empty()) {
      return;
    }
    int columns = cols_ - 1;
    int rows = rows_ - 1;

    std::vector<cv::Point2f> corners;
    for (const Eigen::Vector2d& point : target_pose_points) {
      corners.push_back(cv::Point2f(point.x(), point.y()));
    }

    cv::Scalar color{255, 0, 0};

    // lines left right
    for (size_t y = 0; y < rows - 1; y++) {
      size_t line = y * columns;
      for (size_t x = 0; x < columns - 1; x++) {
        cv::line(image.Data(), corners[line + x], corners[line + x + 1], color, 2);
        cv::line(image.Data(), corners[line + x], corners[line + columns + x], color, 2);
      }
    }
    // right border
    for (size_t i = columns - 1; i < corners.size() - 1; i += columns) {
      cv::line(image.Data(), corners[i], corners[i + columns], color, 2);
    }
    // bottom border
    size_t last_line = (rows - 1) * columns;
    for (size_t i = 0; i < columns - 1; i++) {
      cv::line(image.Data(), corners[last_line + i], corners[last_line + i + 1], color, 2);
    }

    // origin in red
    cv::circle(image.Data(), corners[0], 3, {0, 0, 255}, cv::FILLED);
  }
}
