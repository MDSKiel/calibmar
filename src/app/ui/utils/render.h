#pragma once

#include "calibmar/core/pixmap.h"
#include "colmap/scene/camera.h"
#include <Eigen/Core>
#include <QtGui>

namespace calibmar {
  namespace render {
    void DrawChessboardCorners(Pixmap& pixmap, int columns, int rows, const std::vector<cv::Point2f>& corners);

    void DrawChessboardGrid(Pixmap& pixmap, int columns, int rows, const std::vector<cv::Point2f>& corners);

    void DrawCoordinateAxis(Pixmap& pixmap, const colmap::Camera& camera, const Eigen::Vector4d& rotation,
                            const Eigen::Vector3d& translation, float length);

    // Returns a QImage from a Pixmap. Does not copy and therefore the lifetime is bound to the pixmap.
    QImage GetQImageFromPixmap(const Pixmap& pixmap);
  }
}