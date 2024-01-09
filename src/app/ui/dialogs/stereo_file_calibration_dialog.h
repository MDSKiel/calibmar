#pragma once

#include "ui/widgets/common_calibration_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class StereoFileCalibrationDialog : public QDialog {
   public:
    struct Options {
      ChessboardFeatureExtractor::Options calibration_target_options;
      CameraModelType camera_model;
      std::optional<std::vector<double>> initial_camera_parameters;
      std::string images_directory1;
      std::string images_directory2;
    };

    StereoFileCalibrationDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters();

    QLineEdit* directory_edit1_;
    QLineEdit* directory_edit2_;    
    CameraModelSelectorWidget* camera_model_selector_;
    ChessboardTargetOptionsWidget* calibration_target_options_;
  };
}
