#pragma once

#include <QtCore>
#include <QtWidgets>

#include "calibmar/calibrators/calibrator.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "ui/utils/flow_layout.h"
#include "ui/widgets/extraction_image_widget.h"
#include "ui/widgets/extraction_images_widget.h"

namespace calibmar {

  // Widget that holds extracted images and calibration result widget
  class CalibrationWidget : public QWidget {
   public:
    CalibrationWidget(QWidget* parent = nullptr,
                      const std::function<void(const std::string&, std::pair<int, int>&)> double_click_callback = nullptr);

    void AddExtractionItem(ExtractionImageWidget* widget);
    void StartCalibration();
    void EndCalibration(QWidget* calibration_result);

   private:
    QLayout* main_layout_;
    QWidget* calibration_widget_;
    ExtractionImagesWidget* extraction_images_;
  };
}
