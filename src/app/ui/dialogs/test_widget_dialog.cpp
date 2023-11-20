#include "test_widget_dialog.h"
#include "ui/widgets/image_widget.h"

#include <QtWidgets>
#include <colmap/estimators/pose.h>
#include <colmap/util/misc.h>
#include <filesystem>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calibmar/extractors/chessboard_extractor.h"
#include "calibmar/readers/livestream_reader.h"
#include "ui/widgets/collapsible_widget.h"
#include "ui/widgets/offset_diagram_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

namespace calibmar {

  TestWidgetDialog::TestWidgetDialog(QWidget* parent) : QDialog(parent) {
    QVBoxLayout* layout = new QVBoxLayout(this);

    // pixmap_.Read(R"("")");
    Eigen::Matrix3d camera_mat;
    camera_mat << 1024, 0, 1024, 0, 1024, 768, 0, 0, 1;

    ZoomableScrollArea* area = new ZoomableScrollArea(this);
    area->setFixedHeight(500);
    OffsetDiagramWidget* widget = new OffsetDiagramWidget({0.0132145, 2.33874e-05, -0.0298957}, camera_mat, pixmap_, this);
    area->setWidget(widget);
    area->widget()->resize(pixmap_.Width() * (800.0 / pixmap_.Height()), 800);
    QTimer::singleShot(0, this, [area]() { area->verticalScrollBar()->setValue(area->verticalScrollBar()->maximum()); });

    CollapsibleWidget* w = new CollapsibleWidget("Show Displacement", this);
    w->SetWidget(area, 500);

    layout->addWidget(w, 0, Qt::AlignTop);
  }
}