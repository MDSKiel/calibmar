#pragma once

#include "calibmar/core/calibration.h"
#include "ui/dialogs/file_calibration_dialog.h"
#include "ui/widgets/calibration_widget.h"
#include "ui/dialogs/stereo_file_calibration_dialog.h"

namespace calibmar {

  class StereoFilesCalibrationRunner {
   public:
    StereoFilesCalibrationRunner(CalibrationWidget* calibration_widget, StereoFileCalibrationDialog::Options options);

    bool Run(Calibration& calibration1, Calibration& calibration2);

   private:
    CalibrationWidget* calibration_widget_;
    StereoFileCalibrationDialog::Options options_;
  };
}
