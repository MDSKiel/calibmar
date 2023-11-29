#pragma once

#include "ui/widgets/common_calibration_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class FileCalibrationDialog : public QDialog {
   public:
    struct Options {
      CalibrationTargetOptionsWidget::Options calibration_target_options;
      CameraModelType camera_model;
      std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration;
      std::optional<std::vector<double>> initial_camera_parameters;
      std::string images_directory;
    };

    FileCalibrationDialog(QWidget* parent = nullptr);

    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters();

    QLineEdit* directory_edit_;
    CommonCalibrationOptionsWidget* calibration_options_widget_;
  };
}
