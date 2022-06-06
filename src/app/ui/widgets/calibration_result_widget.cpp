#include "calibration_result_widget.h"

#include "calibmar/core/report.h"
#include "ui/widgets/collapsible_widget.h"
#include "ui/widgets/offset_diagram_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include <iomanip>
#include <iostream>
#include <regex>

namespace calibmar {
  CalibrationResultWidget::CalibrationResultWidget(Calibration& calibration, std::unique_ptr<Pixmap> offset_visu_pixmap,
                                                   QWidget* parent)
      : QWidget(parent), offset_visu_pixmap_(std::move(offset_visu_pixmap)) {
    QVBoxLayout* layout = new QVBoxLayout(this);
    AddResultText(report::GenerateResultString(calibration), layout);

    if (calibration.Camera().NonSvpModelId() == colmap::DoubleLayerSphericalRefractiveInterface::kNonSvpModelId &&
        offset_visu_pixmap_) {
      std::vector<double>& params = calibration.Camera().NonSvpParams();
      OffsetDiagramWidget* offset_widget = new OffsetDiagramWidget(
          Eigen::Vector3d(params[0], params[1], params[2]), calibration.Camera().CalibrationMatrix(), *offset_visu_pixmap_, this);

      ZoomableScrollArea* area = new ZoomableScrollArea(this);
      area->setWidget(offset_widget);
      area->widget()->resize(offset_visu_pixmap_->Width() * (800.0 / offset_visu_pixmap_->Height()), 800);
      QTimer::singleShot(50, area, [area]() { area->verticalScrollBar()->setValue(area->verticalScrollBar()->maximum()); });
      CollapsibleWidget* collapse = new CollapsibleWidget("Show Displacement", this);
      collapse->SetWidget(area, 500);

      layout->addWidget(collapse);
    }
  }

  CalibrationResultWidget::CalibrationResultWidget(const std::string& message, QWidget* parent) : QWidget(parent) {
    QHBoxLayout* layout = new QHBoxLayout(this);
    AddResultText(message, layout);
  }

  void CalibrationResultWidget::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);

    result_text_->setFixedHeight(result_text_->document()->size().height() + 20);
  }

  void CalibrationResultWidget::AddResultText(const std::string& message, QLayout* layout) {
    QString text = QString::fromStdString(message);
    result_text_ = new QTextEdit(this);
    result_text_->setWordWrapMode(QTextOption::NoWrap);
    result_text_->setFontFamily("Courier New");
    result_text_->setPlainText(text);
    layout->addWidget(result_text_);
  }
}