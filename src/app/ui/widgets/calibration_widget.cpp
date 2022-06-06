#include "ui/widgets/calibration_widget.h"

namespace calibmar {
  CalibrationWidget::CalibrationWidget(QWidget* parent,
                                       const std::function<void(const std::string&, std::pair<int, int>&)> double_click_callback)
      : QWidget(parent) {
    main_layout_ = new QVBoxLayout(this);
    extraction_images_ = new ExtractionImagesWidget(this, double_click_callback);

    QGroupBox* features_groupbox = new QGroupBox("Extracted Features");
    QVBoxLayout* features_groupbox_layout = new QVBoxLayout(features_groupbox);
    features_groupbox_layout->addWidget(extraction_images_);
    main_layout_->addWidget(features_groupbox);


    calibration_widget_ = new QGroupBox("Calibration");
    QVBoxLayout* calibration_layout = new QVBoxLayout(calibration_widget_);
    calibration_widget_->setVisible(false);
    main_layout_->addWidget(calibration_widget_);
  }

  void CalibrationWidget::AddExtractionItem(ExtractionImageWidget* widget) {
    extraction_images_->AddImage(widget);
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

}
