#include "ui/widgets/calibration_widget.h"
#include "calibration_widget.h"

namespace calibmar {
  CalibrationWidget::CalibrationWidget(
      QWidget* parent, const std::function<void(const std::string&, const class TargetVisualizer&)> double_click_callback,
      const std::function<const std::vector<std::string>(const SortOrder&)> sorting_callback)
      : QWidget(parent), calibration_ended_(false), sorting_callback_(sorting_callback) {
    main_layout_ = new QVBoxLayout(this);
    extraction_images_ = new ExtractionImagesWidget(this, double_click_callback);

    QGroupBox* features_groupbox = new QGroupBox("Extracted Features");
    QVBoxLayout* features_groupbox_layout = new QVBoxLayout(features_groupbox);
    if (sorting_callback_) {
      // one for top and one for bottom
      for (size_t i = 0; i < 2; i++) {
        QWidget* sorting_widget = new QWidget();
        QHBoxLayout* sorting_layout = new QHBoxLayout(sorting_widget);
        sorting_layout->setAlignment(Qt::AlignLeft);
        QLabel* sorting_label = new QLabel();
        sorting_label->setText("Sort by:");
        QComboBox* sorting_combobox = new QComboBox(this);
        sorting_combobox->addItems({"Name Descending", "Name Ascending", "RMS Descending", "RMS Ascending"});
        sorting_combobox->setCurrentIndex(1);

        connect(sorting_combobox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
          for (auto widget : this->sorting_widgets_) {
            // Keep both sorting widgets synchronized
            QComboBox* combobox = widget->findChild<QComboBox*>();
            combobox->blockSignals(true);
            combobox->setCurrentIndex(index);
            combobox->blockSignals(false);
          }
          this->SetExtractionImageOrder(index);
        });

        sorting_layout->addWidget(sorting_label);
        sorting_layout->addWidget(sorting_combobox);
        sorting_widget->setVisible(false);
        QMargins margins = sorting_layout->contentsMargins();
        if (i == 0) {
          margins.setBottom(0);
        }
        else {
          margins.setTop(0);
        }

        sorting_layout->setContentsMargins(margins);
        sorting_widgets_.push_back(sorting_widget);
      }
    }
    features_groupbox_layout->setContentsMargins(0, 0, 0, 0);

    features_groupbox_layout->addWidget(sorting_widgets_[0]);
    features_groupbox_layout->addWidget(extraction_images_);
    features_groupbox_layout->addWidget(sorting_widgets_[1]);

    main_layout_->addWidget(features_groupbox);

    calibration_widget_ = new QGroupBox("Calibration");
    QVBoxLayout* calibration_layout = new QVBoxLayout(calibration_widget_);
    calibration_widget_->setVisible(false);
    main_layout_->addWidget(calibration_widget_);
  }

  void CalibrationWidget::AddExtractionItem(QWidget* widget, const std::string& key) {
    // if EndCalibration is called this might be null (can happen with unfortunate ordering in deferred qt calls)
    if (!calibration_ended_) {
      extraction_images_->AddExtractionItem(widget, key);
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
    calibration_ended_ = true;

    if (calibration_widget_) {
      delete calibration_widget_;
    }

    calibration_widget_ = new QGroupBox("Calibration");
    QVBoxLayout* calibration_layout = new QVBoxLayout(calibration_widget_);
    calibration_layout->addWidget(calibration_result);
    main_layout_->addWidget(calibration_widget_);
    if (extraction_images_->children().size() > 1 && sorting_callback_) {
      for (auto widget : sorting_widgets_) widget->setVisible(true);
    }
  }

  void CalibrationWidget::SetTargetVisualizer(std::unique_ptr<class TargetVisualizer> target_visualizer) {
    target_visualizer_ = std::move(target_visualizer);
  }

  void CalibrationWidget::SetPerViewRms(const std::vector<Image>& images, const std::vector<double>& per_view_rms) {
    std::unordered_map<std::string, double> per_view_rms_map;
    for (size_t i = 0; i < images.size(); i++) {
      per_view_rms_map[images[i].Name()] = per_view_rms[i];
    }
    extraction_images_->SetPerViewRMS(per_view_rms_map);
  }

  void CalibrationWidget::SetPerViewRms(const std::vector<std::vector<Image>*>& images,
                                        const std::vector<std::vector<double>*>& per_view_rms) {
    std::unordered_map<std::string, double> per_view_rms_map;
    for (size_t i = 0; i < images.size(); i++) {
      const auto& image_set = images[i];
      const auto& rms_set = per_view_rms[i];
      for (size_t i = 0; i < image_set->size(); i++) {
        per_view_rms_map[image_set->at(i).Name()] = rms_set->at(i);
      }
    }
    extraction_images_->SetPerViewRMS(per_view_rms_map);
  }

  const class TargetVisualizer& CalibrationWidget::TargetVisualizer() {
    return *target_visualizer_;
  }

  void CalibrationWidget::SetExtractionImageOrder(int index) {
    if (!sorting_callback_) {
      return;
    }
    SortOrder sort_order{index};
    const std::vector<std::string> order = sorting_callback_(sort_order);
    extraction_images_->SetOrder(order);
  }
}
