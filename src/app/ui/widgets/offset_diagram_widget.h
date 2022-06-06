#pragma once

#include "calibmar/core/pixmap.h"

#include <QtCore>
#include <QtWidgets>

#include <Eigen/Core>

namespace calibmar {
  // Displays a offset diagram, showing dome port housing offset
  class OffsetDiagramWidget : public QWidget {
   public:
    OffsetDiagramWidget(const Eigen::Vector3d& offset, const Eigen::Matrix3d& camera_matrix, const Pixmap& img,
                        QWidget* parent = nullptr);

   protected:
    void paintEvent(QPaintEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);

   private:
    void AddLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const QColor& color = QColor(0, 0, 0));
    void AddPoints(const Eigen::Matrix3Xd& new_points, const QColor& color = QColor(0, 0, 0));

    bool dragging_;
    QRectF drag_area_;
    QRectF drag_area_scaled_;
    QPointF drag_offset_scaled_;
    QPointF drag_target_;
    double axis_max_value_;
    double current_scale_;

    QPen pen_;
    QImage img_;

    Eigen::Matrix4Xd points_h_;
    Eigen::Matrix4Xd tick_positions_h_;
    std::vector<double> ticks_;
    Eigen::Matrix3Xd points_;
    std::vector<QColor> colors_;
    Eigen::Matrix<double, 3, 4> projection_;
    Eigen::Vector3d offset_;

    const QColor x_color = QColorConstants::Red;
    const QColor y_color = QColorConstants::Green;
    const QColor z_color = QColorConstants::Blue;
  };
}