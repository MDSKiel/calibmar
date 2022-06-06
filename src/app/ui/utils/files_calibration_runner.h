#pragma once

#include "calibmar/core/calibration.h"
#include "ui/dialogs/file_calibration_dialog.h"
#include "ui/widgets/calibration_widget.h"

namespace calibmar {

  class FilesCalibrationRunner {
   public:
    FilesCalibrationRunner(CalibrationWidget* calibration_widget, FileCalibrationDialog::Options options);

    bool Run(Calibration& calibration);

   private:
    CalibrationWidget* calibration_widget_;
    FileCalibrationDialog::Options options_;

    std::unique_ptr<Pixmap> last_pixmap_;
  };
}
