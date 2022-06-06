#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/pixmap.h"

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  // Widget that holds the calibration result (e.g. report or error message)
  class CalibrationResultWidget : public QWidget {
   public:
    CalibrationResultWidget(Calibration& calibration, std::unique_ptr<Pixmap> offset_visu_pixmap = std::unique_ptr<Pixmap>(),
                            QWidget* parent = nullptr);

    CalibrationResultWidget(const std::string& message, QWidget* parent = nullptr);

   protected:
    virtual void showEvent(QShowEvent* e) override;

   private:
    void AddResultText(const std::string& message, QLayout* layout);

    std::unique_ptr<Pixmap> offset_visu_pixmap_;
    QTextEdit* result_text_;
  };
}