#include "offset_diagram_widget.h"
#include "ui/utils/render.h"

#include <Eigen/Geometry>
#include <Eigen/SparseCore>

namespace calibmar {

  OffsetDiagramWidget::OffsetDiagramWidget(const Eigen::Vector3d& offset, const Eigen::Matrix3d& camera_matrix, const Pixmap& img,
                                           QWidget* parent)
      : QWidget(parent),
        pen_(QPen(QBrush(Qt::black), img.Height() / 180, Qt::SolidLine, Qt::RoundCap)),
        dragging_(false),
        offset_(offset) {
    setMouseTracking(true);
    setMinimumSize(100, 100);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    img_ = render::GetQImageFromPixmap(img);

    double arrow_angle = 30 * (M_PI / 180);
    double arrow_length_main = 0.05;
    double arrow_length_second = std::tan(arrow_angle) * arrow_length_main;
    double tick_size = 0.025;

    // axis
    AddLine({-1, 0, 0}, {1, 0, 0});
    // arrow
    AddLine({1, 0, 0}, {1 - arrow_length_main, arrow_length_second, 0});
    AddLine({1, 0, 0}, {1 - arrow_length_main, -arrow_length_second, 0});
    // ticks at 1/3 steps
    for (double i = -0.666; i < 0.8; i += 0.333) {
      AddLine({i, -tick_size, 0}, {i, tick_size, 0});
    }
    // create other axis by copy and rotate
    Eigen::Matrix3Xd x_axis = points_;
    Eigen::Matrix3d rot_z(Eigen::AngleAxisd(-90 * (M_PI / 180), Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rot_y(Eigen::AngleAxisd(-90 * (M_PI / 180), Eigen::Vector3d::UnitY()));
    Eigen::Matrix3d rot_yz = rot_z * rot_y;
    Eigen::Matrix3Xd y_axis = rot_z * x_axis;
    Eigen::Matrix3Xd z_axis = rot_yz * x_axis;
    AddPoints(y_axis);
    AddPoints(z_axis);
    // tick labels only on x
    Eigen::Matrix<double, 3, 2> tick_positions = (Eigen::Matrix<double, 3, 2>() << x_axis.col(13), x_axis.col(15)).finished();

    // offset vectors
    // find the axis value scale so that the max offset value is at about 80% of axis length
    double max_offset = offset.cwiseAbs().maxCoeff();
    axis_max_value_ = max_offset * 1.2;
    // to ensure the ticks (which are at thirds) are 3 digit precision floats
    axis_max_value_ = (std::ceil((axis_max_value_ * 1000) / 3) * 3) / 1000;
    // scale offset values to match the scaled axis
    Eigen::Vector3d offset_scaled = offset * (1 / axis_max_value_);
    // tick labels
    ticks_.push_back(axis_max_value_ / 3.0);
    ticks_.push_back(axis_max_value_ / 3.0 * 2);
    // create all offsets on x axis and rotate after
    Eigen::Matrix3Xd offsets(3, 6 * 3);
    for (size_t i = 0; i < 3; i++) {
      size_t i_off = i * 6;
      double offset = offset_scaled(i);
      offsets.col(i_off + 0) << 0, 0, 0;
      offsets.col(i_off + 1) << offset, 0, 0;
      // arrow head
      double arrow_main = std::copysign(arrow_length_main, offset);
      offsets.col(i_off + 2) << offset, 0, 0;
      offsets.col(i_off + 3) << offset - arrow_main, arrow_length_second, 0;
      offsets.col(i_off + 4) << offset, 0, 0;
      offsets.col(i_off + 5) << offset - arrow_main, -arrow_length_second, 0;
    }
    // rotate respective y and z offsets
    AddPoints(offsets.leftCols(6), x_color);
    AddPoints(rot_z * offsets.middleCols(6, 6), y_color);
    AddPoints(rot_yz * offsets.rightCols(6), z_color);

    projection_ = (Eigen::Matrix<double, 3, 4>() << camera_matrix, Eigen::Vector3d::Zero()).finished();

    // average f
    double f = (camera_matrix(0) + camera_matrix(4)) / 2;
    // target size of 50% of smaller img edge
    double scale = std::min(img_.width(), img_.height()) * 0.25 / f;
    points_ *= scale;
    // scale points
    tick_positions_h_ = (tick_positions * scale).colwise().homogeneous();
    points_h_ = points_.colwise().homogeneous();
    drag_target_ = QPointF(0, img_.height() - (2 * scale) * f);
  }

  void OffsetDiagramWidget::paintEvent(QPaintEvent* event) {
    double half_width = points_.col(1)(0);
    double f = (projection_(0) + projection_(4)) / 2;
    double cx = projection_(6);
    double cy = projection_(7);
    // translate to match drag_target_ in pixel and onto image pane
    double tx = (-cx / f) + half_width + drag_target_.x() / f;
    double ty = (-cy / f) + half_width + drag_target_.y() / f;
    double tz = 1;
    Eigen::Matrix4d translation = Eigen::Affine3d(Eigen::Translation3d(tx, ty, tz)).matrix();
    // translate & project
    Eigen::Matrix2Xd points = (projection_ * translation * points_h_).colwise().hnormalized();
    Eigen::Matrix2Xd ticks = (projection_ * translation * tick_positions_h_).colwise().hnormalized();
    // determine current diagram area from axis coordinates
    double left = points.col(0)(0);
    double right = points.col(1)(0);
    double top = points.col(17)(1);
    double bottom = points.col(16)(1);
    drag_area_ = QRectF(left, top, right - left, bottom - top);

    QPainter painter(this);
    // respect widget scaling wile keeping aspect ratio
    double width = this->width();
    double height = this->height();
    current_scale_ = std::min(width / img_.width(), height / img_.height());
    painter.scale(current_scale_, current_scale_);
    //  for hitbox only
    drag_area_scaled_ = QRectF(drag_area_.x() * current_scale_, drag_area_.y() * current_scale_,
                               drag_area_.width() * current_scale_, drag_area_.height() * current_scale_);

    painter.drawImage(0, 0, img_);
    // paint diagram only inside image
    painter.setClipRect(0, 0, img_.width(), img_.height());
    painter.setRenderHint(QPainter::RenderHint::Antialiasing);
    QPainterPath path;
    path.addRoundedRect(drag_area_, 5, 5);
    painter.fillPath(path, QBrush(QColor(255, 255, 255, 100)));

    // lines
    for (size_t i = 0; i < points.cols(); i += 2) {
      pen_.setColor(colors_[i / 2]);
      painter.setPen(pen_);
      QLineF line(points.col(i)(0), points.col(i)(1), points.col(i + 1)(0), points.col(i + 1)(1));
      painter.drawLine(line);
    }

    // tick labels
    painter.setPen(QColor(0, 0, 0));
    QFont font = painter.font();
    font.setPixelSize(img_.height() / 45);
    font.setBold(true);
    painter.setFont(font);
    QFontMetrics fm(painter.font());
    QString t1_label = QString::number(axis_max_value_ / 3.0);
    QString t2_label = QString::number(axis_max_value_ / 3.0 * 2);
    for (size_t i = 0; i < ticks.cols(); i++) {
      QPointF pos(ticks.col(i).x() - fm.width(t1_label) / 2, ticks.col(i).y() + fm.height());
      painter.drawText(pos, QString::number(ticks_[i]));
    }

    QPointF label_pos(drag_area_.topLeft() + QPointF(fm.horizontalAdvance("x") / 2, fm.height()));
    painter.setPen(x_color);
    painter.drawText(label_pos, QString("x: %1").arg(offset_.x()));
    label_pos += QPointF(0, fm.height());
    painter.setPen(y_color);
    painter.drawText(label_pos, QString("y: %1").arg(offset_.y()));
    label_pos += QPointF(0, fm.height());
    painter.setPen(QColor(z_color));
    painter.drawText(label_pos, QString("z: %1").arg(offset_.z()));
  }

  void OffsetDiagramWidget::mouseMoveEvent(QMouseEvent* event) {
    if (drag_area_scaled_.contains(event->localPos())) {
      if (!dragging_) {
        setCursor(QCursor(Qt::OpenHandCursor));
      }
    }
    else {
      setCursor(QCursor());
    }

    if (dragging_) {
      // determine (unscaled)target diagram position, while staying inside image bounds
      int limit_x1 = -drag_area_.width() / 2;
      int limit_y1 = -drag_area_.height() / 2;
      int limit_x2 = img_.width() - drag_area_.width() / 2;
      int limit_y2 = img_.height() - drag_area_.height() / 2;
      // localPos is in actual (scaled) widget coordinates
      drag_target_ = (event->localPos() + drag_offset_scaled_) / current_scale_;
      drag_target_.setX(std::clamp(static_cast<int>(drag_target_.x()), limit_x1, limit_x2));
      drag_target_.setY(std::clamp(static_cast<int>(drag_target_.y()), limit_y1, limit_y2));
      update();
    }
    QWidget::mouseMoveEvent(event);
  }

  void OffsetDiagramWidget::mousePressEvent(QMouseEvent* event) {
    dragging_ = drag_area_scaled_.contains(event->localPos());
    if (dragging_) {
      setCursor(QCursor(Qt::ClosedHandCursor));
      drag_offset_scaled_ = drag_area_scaled_.topLeft() - event->localPos();
    }

    QWidget::mousePressEvent(event);
  }

  void OffsetDiagramWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (dragging_) {
      setCursor(QCursor(Qt::OpenHandCursor));
    }
    dragging_ = false;
    QWidget::mouseReleaseEvent(event);
  }

  void OffsetDiagramWidget::AddLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const QColor& color) {
    Eigen::Index new_size = points_.cols() + 2;
    points_.conservativeResize(Eigen::NoChange, new_size);
    points_.col(new_size - 2) = p1;
    points_.col(new_size - 1) = p2;
    colors_.push_back(color);
  }
  void OffsetDiagramWidget::AddPoints(const Eigen::Matrix3Xd& new_points, const QColor& color) {
    points_.conservativeResize(Eigen::NoChange, points_.cols() + new_points.cols());
    points_.rightCols(new_points.cols()) = new_points;

    for (size_t i = 0; i < new_points.cols() / 2; i++) {
      colors_.push_back(color);
    }
  }
}