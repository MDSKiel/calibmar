#include "calibration_result_widget.h"

#include "calibmar/core/report.h"
#include "ui/widgets/collapsible_widget.h"
#include "ui/widgets/offset_diagram_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include <colmap/ui/model_viewer_widget.h>

#include <iomanip>
#include <iostream>
#include <regex>

namespace calibmar {
  CalibrationResultWidget::CalibrationResultWidget(Calibration& calibration, std::unique_ptr<Pixmap> offset_visu_pixmap,
                                                   std::shared_ptr<colmap::Reconstruction> reconstruction, QWidget* parent)
      : QWidget(parent), offset_visu_pixmap_(std::move(offset_visu_pixmap)) {
    QVBoxLayout* layout = new QVBoxLayout(this);
    AddResultText(report::GenerateResultString(calibration), layout);

    // only show offset diagramm with dome port
    if (calibration.Camera().RefracModelId() == colmap::DomePort::kRefracModelId && offset_visu_pixmap_) {
      std::vector<double>& params = calibration.Camera().RefracParams();
      OffsetDiagramWidget* offset_widget = new OffsetDiagramWidget(
          Eigen::Vector3d(params[0], params[1], params[2]), calibration.Camera().CalibrationMatrix(), *offset_visu_pixmap_, this);

      ZoomableScrollArea* area = new ZoomableScrollArea(this);
      area->setWidget(offset_widget);
      area->widget()->resize(offset_visu_pixmap_->Width() * (800.0 / offset_visu_pixmap_->Height()), 800);
      QTimer::singleShot(50, area, [area]() { area->verticalScrollBar()->setValue(area->verticalScrollBar()->maximum()); });
      CollapsibleWidget* collapse = new CollapsibleWidget("Show Displacement", nullptr, this);
      collapse->SetWidget(area, 500);

      layout->addWidget(collapse);
    }

    if (reconstruction != nullptr) {
      options_manager_ = std::make_unique<colmap::OptionManager>();
      colmap::ModelViewerWidget* model_viewer_widget = new colmap::ModelViewerWidget(this, options_manager_.get());
      model_viewer_widget->statusbar_status_label = new QLabel("0 Images - 0 Points", this);
      model_viewer_widget->statusbar_status_label->setVisible(false);
      model_viewer_widget->reconstruction = reconstruction;
      CollapsibleWidget* collapse = new CollapsibleWidget(
          "Reconstruction",
          [model_viewer_widget](bool visible) {
        if (visible) {
          model_viewer_widget->ReloadReconstruction();
        }
      },
          this);
      collapse->SetWidget(model_viewer_widget, 800);
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