#include "housing_diagram_widget.h"

namespace calibmar {
  HousingDiagramWidget::HousingDiagramWidget(QWidget* parent, QSize default_size) {
    SetupDiagram(DiagramType::FlatPort, default_size);
  }

  void HousingDiagramWidget::SetFOVDegrees(double fov_degrees) {
    diagram_->SetFOVDegrees(fov_degrees);
    diagram_->CalculatePoints();
  }

  void HousingDiagramWidget::SetRayCount(int ray_count) {
    diagram_->SetRayCount(ray_count);
    diagram_->CalculatePoints();
  }

  void HousingDiagramWidget::SetRefrIndexAir(double na) {
    diagram_->SetRefractiveIndexAir(na);
    diagram_->CalculatePoints();
  }

  void HousingDiagramWidget::SetRefrIndexGlass(double ng) {
    diagram_->SetRefractiveIndexGlass(ng);
    diagram_->CalculatePoints();
  }

  void HousingDiagramWidget::SetRefrIndexWater(double nw) {
    diagram_->SetRefractiveIndexWater(nw);
    diagram_->CalculatePoints();
  }

  void HousingDiagramWidget::SetDiagramType(DiagramType type) {
    SetupDiagram(type, base_size_);
    diagram_->CalculatePoints();
  }

  void HousingDiagramWidget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    diagram_->Draw(&painter);
  }

  void HousingDiagramWidget::mousePressEvent(QMouseEvent* event) {
    dragging_start_ = handle_start_.contains(event->pos());
    dragging_end_ = handle_end_.contains(event->pos());
  }

  void HousingDiagramWidget::mouseReleaseEvent(QMouseEvent* event) {
    dragging_start_ = false;
    dragging_end_ = false;
  }

  void HousingDiagramWidget::mouseMoveEvent(QMouseEvent* event) {
    int center_x = event->pos().x();
    int center_y = event->pos().y();
    if (dragging_start_ || dragging_end_) {
      if (dragging_start_) {
        QPointF delta = handle_end_.center() - handle_start_.center();
        handle_start_.moveCenter(event->pos());
        diagram_->SetHandleStart({handle_start_.center().x(), handle_start_.center().y()});
        if (diagram_type_ == DiagramType::DomePort) {
          // for dome move both
          handle_end_.moveCenter(event->pos() + delta);
          diagram_->SetHandleEnd({handle_end_.center().x(), handle_end_.center().y()});
        }
      }
      if (dragging_end_) {
        handle_end_.moveCenter(event->pos());
        diagram_->SetHandleEnd({handle_end_.center().x(), handle_end_.center().y()});
      }
      diagram_->CalculatePoints();
      update();
    }
  }

  void HousingDiagramWidget::resizeEvent(QResizeEvent* event) {
    double ratio_w = event->size().width() / static_cast<double>(base_size_.width());
    double ratio_h = event->size().height() / static_cast<double>(base_size_.height());
    diagram_->Scale(std::min(ratio_w, ratio_h));
    diagram_->GetCurrentHandles(&handle_start_, &handle_end_);
    update();
  }

  void HousingDiagramWidget::SetupDiagram(DiagramType type, const QSize& size) {
    diagram_type_ = type;
    int width = size.width();
    int height = size.height();
    base_size_ = size;

    int handle_start_center_x = width / 2;
    int handle_start_center_y = height / 2;
    Eigen::Vector2d camera_center{width / 2, height / 4};
    std::unique_ptr<HousingDiagram> diagram;
    Eigen::Matrix3d transformation = diagram_ ? diagram_->GetTransformation() : Eigen::Matrix3d::Identity();
    switch (type) {
      case DiagramType::FlatPort:
        diagram = std::make_unique<FlatPortDiagram>(transformation);
        break;
      case DiagramType::DomePort:
        diagram = std::make_unique<DomePortDiagram>(transformation);
        handle_start_center_x = camera_center.x();
        handle_start_center_y = camera_center.y();
        break;
    }

    diagram_ = std::move(diagram);
    diagram_->InitCamera(camera_center, 75);
    handle_start_ = QRectF(handle_start_center_x - handle_radius_, handle_start_center_y - handle_radius_, handle_radius_ * 2,
                           handle_radius_ * 2);

    handle_start_center_y += 200;
    handle_end_ = QRectF(handle_start_center_x - handle_radius_, handle_start_center_y - handle_radius_, handle_radius_ * 2,
                         handle_radius_ * 2);

    diagram_->InitHandle({handle_start_.center().x(), handle_start_.center().y()},
                         {handle_end_.center().x(), handle_end_.center().y()}, handle_radius_ * 2);
    diagram_->InitSize({width, height});
    diagram_->InitInterfaceThickness(30);
    diagram_->SetRayCount(8);
    diagram_->CalculatePoints();
    diagram_->GetCurrentHandles(&handle_start_, &handle_end_);
  }
}