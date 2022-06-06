#pragma once

#include "calibmar/calibrators/calibrator.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "calibmar/readers/livestream_reader.h"
#include "ui/widgets/chessboard_target_widget.h"
#include "ui/widgets/housing_selector_widget.h"
#include "ui/widgets/model_selector_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class TestWidgetDialog : public QDialog {
   public:
    TestWidgetDialog(QWidget* parent = nullptr);

   private:
    Pixmap pixmap_;
  };
}
