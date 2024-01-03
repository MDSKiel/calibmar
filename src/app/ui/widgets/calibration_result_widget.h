#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/pixmap.h"

#include <colmap/controllers/option_manager.h>
#include <colmap/scene/reconstruction.h>

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  // Widget that holds the calibration result (e.g. report or error message)
  class CalibrationResultWidget : public QWidget {
   public:
    CalibrationResultWidget(Calibration& calibration, std::unique_ptr<Pixmap> offset_visu_pixmap = std::unique_ptr<Pixmap>(),
                            std::shared_ptr<colmap::Reconstruction> reconstruction = nullptr, QWidget* parent = nullptr);

    CalibrationResultWidget(const std::string& message, QWidget* parent = nullptr);

   protected:
    virtual void showEvent(QShowEvent* e) override;

   private:
    std::unique_ptr<Pixmap> offset_visu_pixmap_;
    std::unique_ptr<colmap::OptionManager> options_manager_;
    QTextEdit* result_text_;
  };
}