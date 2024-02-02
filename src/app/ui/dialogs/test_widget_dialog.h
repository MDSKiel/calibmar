#pragma once

#include "calibmar/calibrators/calibrator.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "calibmar/readers/livestream_reader.h"
#include "ui/widgets/calibration_target_widget.h"
#include "ui/widgets/housing_selector_widget.h"
#include "ui/widgets/camera_model_selector_widget.h"

#include <colmap/controllers/option_manager.h>

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class TestWidgetDialog : public QDialog {
   public:
    TestWidgetDialog(QWidget* parent = nullptr);

   private:
    Pixmap pixmap_;
    std::unique_ptr<colmap::OptionManager> options_manager_ = std::make_unique<colmap::OptionManager>();
  };
}
