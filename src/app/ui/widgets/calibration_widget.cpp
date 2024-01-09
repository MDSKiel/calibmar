#include "ui/widgets/calibration_widget.h"
#include "calibration_widget.h"

namespace calibmar {
  CalibrationWidget::CalibrationWidget(
      QWidget* parent, const std::function<void(const std::string&, const class TargetVisualizer&)> double_click_callback)
      : QWidget(parent) {
    main_layout_ = new QVBoxLayout(this);
    extraction_images_ = new ExtractionImagesWidget(this, double_click_callback);

    QGroupBox* features_groupbox = new QGroupBox("Extracted Features");
    QVBoxLayout* features_groupbox_layout = new QVBoxLayout(features_groupbox);
    features_groupbox_layout->addWidget(extraction_images_);
    features_groupbox_layout->setContentsMargins(0, 0, 0, 0);
    main_layout_->addWidget(features_groupbox);

    calibration_widget_ = new QGroupBox("Calibration");
    QVBoxLayout* calibration_layout = new QVBoxLayout(calibration_widget_);
    calibration_widget_->setVisible(false);
    main_layout_->addWidget(calibration_widget_);
  }

  void CalibrationWidget::AddExtractionItem(QWidget* widget) {
    // if EndCalibration is called this might be null (can happen with unfortunate ordering in deferred qt calls)
    if (extraction_images_) {
      extraction_images_->AddImage(widget);
    }
  }

  void CalibrationWidget::StartCalibration() {
    calibration_widget_->setVisible(true);

    QProgressBar* progress_bar = new QProgressBar();
    progress_bar->setTextVisible(false);
    progress_bar->setMaximum(0);
    progress_bar->setMinimum(0);
    progress_bar->show();
    calibration_widget_->layout()->addWidget(progress_bar);
  }

  void CalibrationWidget::EndCalibration(QWidget* calibration_result) {
    if (calibration_widget_) {
      delete calibration_widget_;
    }

    calibration_widget_ = new QGroupBox("Calibration");
    QVBoxLayout* calibration_layout = new QVBoxLayout(calibration_widget_);
    calibration_layout->addWidget(calibration_result);
    main_layout_->addWidget(calibration_widget_);
  }

  void CalibrationWidget::SetTargetVisualizer(std::unique_ptr<class TargetVisualizer> target_visualizer) {
    target_visualizer_ = std::move(target_visualizer);
  }

  const class TargetVisualizer& CalibrationWidget::TargetVisualizer() {
    return *target_visualizer_;
  }
}
