#include "render.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace calibmar {
  namespace render {
    void DrawChessboardCorners(Pixmap& pixmap, int columns, int rows, const std::vector<cv::Point2f>& corners) {
      if (corners.empty()) {
        return;
      }

      cv::Mat& cornerMat = pixmap.Data();
      if (cornerMat.channels() == 1) {
        // reallocate in place, this is supported by opencv, the old one gets destroyed if refrence count == 0
        cv::cvtColor(cornerMat, cornerMat, cv::COLOR_GRAY2RGB);
      }

      cv::drawChessboardCorners(cornerMat, {columns, rows}, corners, true);
    }

    void DrawChessboardGrid(Pixmap& pixmap, int columns, int rows, const std::vector<cv::Point2f>& corners) {
      if (corners.empty()) {
        return;
      }
      cv::Scalar color{255, 0, 0};

      // lines left right
      for (size_t y = 0; y < rows - 1; y++) {
        size_t line = y * columns;
        for (size_t x = 0; x < columns - 1; x++) {
          cv::line(pixmap.Data(), corners[line + x], corners[line + x + 1], color, 2);
          cv::line(pixmap.Data(), corners[line + x], corners[line + columns + x], color, 2);
        }
      }
      // right border
      for (size_t i = columns - 1; i < corners.size() - 1; i += columns) {
        cv::line(pixmap.Data(), corners[i], corners[i + columns], color, 2);
      }
      // bottom border
      size_t last_line = (rows - 1) * columns;
      for (size_t i = 0; i < columns - 1; i++) {
        cv::line(pixmap.Data(), corners[last_line + i], corners[last_line + i + 1], color, 2);
      }

      // origin in red
      cv::circle(pixmap.Data(), corners[0], 3, {0, 0, 255}, cv::FILLED);
    }

    void DrawCoordinateAxis(Pixmap& pixmap, const colmap::Camera& camera, const Eigen::Vector4d& rotation,
                            const Eigen::Vector3d& translation, float length) {
      cv::Mat camera_mat;
      cv::eigen2cv(camera.CalibrationMatrix(), camera_mat);
      std::vector<double> distortion_params;
      for (size_t idx : camera.ExtraParamsIdxs()) {
        distortion_params.push_back(camera.Params()[idx]);
      }

      Eigen::AngleAxisd angle_axis(Eigen::Quaterniond(rotation(0), rotation(1), rotation(2), rotation(3)));
      Eigen::Vector3d rot = angle_axis.axis() * angle_axis.angle();

      cv::drawFrameAxes(pixmap.Data(), camera_mat, distortion_params, cv::Vec3f(rot.x(), rot.y(), rot.y()),
                        cv::Vec3f(translation.x(), translation.y(), translation.z()), length);
    }

    QImage GetQImageFromPixmap(const Pixmap& pixmap) {
      const cv::Mat& mat = pixmap.Data();
      QImage::Format format = mat.channels() == 1 ? QImage::Format::Format_Grayscale8 : QImage::Format::Format_BGR888;
      return QImage(mat.data, mat.cols, mat.rows, mat.step, format);
    }
  }
}