#pragma once

#include <QtCore>
#include <QtWidgets>

#include "ui/utils/calibration_target_visualization.h"
#include "ui/widgets/extraction_image_widget.h"
#include "ui/widgets/extraction_images_widget.h"

namespace calibmar {

  // Widget that holds extracted images and calibration result widget
  class CalibrationWidget : public QWidget {
   public:
    CalibrationWidget(QWidget* parent = nullptr,
                      const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback = nullptr);

    void AddExtractionItem(ExtractionImageWidget* widget);
    void StartCalibration();
    void EndCalibration(QWidget* calibration_result);
    void SetTargetVisualizer(std::unique_ptr<TargetVisualizer> target_visualizer);
    const class TargetVisualizer& TargetVisualizer();

   private:
    std::unique_ptr<class TargetVisualizer> target_visualizer_; // basically only holds this for ownership reasons. Child ExtractionImageWidgets use a reference to this.
    QLayout* main_layout_;
    QWidget* calibration_widget_;
    ExtractionImagesWidget* extraction_images_;
  };
}
