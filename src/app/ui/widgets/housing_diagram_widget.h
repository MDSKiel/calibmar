#pragma once

#include <QtCore>
#include <QtWidgets>

#include "ui/utils/housing_diagram.h"

namespace calibmar {
  // Displays extraction images in a flow layout and can propagate image name and cols, rows on double click (for image display)
  class HousingDiagramWidget : public QWidget {
   public:
    enum class DiagramType { FlatPort, DomePort };
    HousingDiagramWidget(QWidget* parent = nullptr, QSize default_size = {500, 500});

    void SetFOVDegrees(double fov_degrees);
    void SetRayCount(int ray_count);
    void SetRefrIndexAir(double na);
    void SetRefrIndexGlass(double ng);
    void SetRefrIndexWater(double nw);
    void SetDiagramType(DiagramType type);

   protected:
    void paintEvent(QPaintEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void resizeEvent(QResizeEvent* event);

   private:
    void SetupDiagram(DiagramType type, const QSize& size);

    DiagramType diagram_type_;
    std::unique_ptr<HousingDiagram> diagram_;
    bool dragging_start_;
    bool dragging_end_;

    QRectF handle_start_;
    QRectF handle_end_;
    int handle_radius_ = 3;
    QSize base_size_;
  };
}
